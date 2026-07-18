import time
import threading
import os
import math

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_LOG_DIR = os.environ.get("ROS_LOG_DIR", os.path.join(BASE_DIR, "logs", "ros"))
os.makedirs(ROS_LOG_DIR, exist_ok=True)
os.environ.setdefault("ROS_LOG_DIR", ROS_LOG_DIR)

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, WaypointPush, WaypointClear, CommandInt, CommandLong

CRUISE_ALTITUDE_M = 15.0
STRAIGHT_ROUTE_TOTAL_M = 900.0
# Uçağı hep rota bitişinin ÇOK ötesine (bu mesafeye) yönlendiririz. Böylece
# hiçbir ara noktaya "varıp" onun etrafında loiter (daire) çizmez ve 900 m
# boyunca dümdüz uçar. 900 m kat edilince ayrıca LOITER'a geçirip durdururuz.
STRAIGHT_ROUTE_AIM_AHEAD_M = 5000.0
ROUTE_TARGET_REACHED_M = 40.0  # yol üstündeki cisimlerin üzerinden geçiş toleransı (loglama)
# 0° = Kuzey. Uçak Gazebo'da pistin uzun ekseni (Kuzey/+Y) boyunca doğuyor,
# bu yüzden düz rota da Kuzey'e gider; böylece uçuş pistin üzerinde kalır.
STRAIGHT_ROUTE_HEADING_DEG = 0.0
ROUTE_TARGET_SPECS = [
    ("T-01", 200.0),
    ("T-02", 280.0),
    ("T-03", 330.0),
    ("T-04", 440.0),
    ("T-05", 520.0),
    ("T-06", 600.0),
    ("T-07", 670.0),
    ("T-08", 740.0),
    ("T-09", 810.0),
    ("T-10", 850.0),
]
MAV_CMD_COMPONENT_ARM_DISARM = 400
ARDUPILOT_FORCE_ARM_MAGIC = 21196.0

class UavController(Node):
    def __init__(self):
        super().__init__("uav_control_node")
        
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.wp_push_client = self.create_client(WaypointPush, "/mavros/mission/push")
        self.wp_clear_client = self.create_client(WaypointClear, "/mavros/mission/clear")
        self.cmd_int_client = self.create_client(CommandInt, "/mavros/cmd/command_int")
        self.cmd_long_client = self.create_client(CommandLong, "/mavros/cmd/command")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.pose_callback,
            qos_profile
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_callback,
            qos_profile
        )

        # Operatör devralma kanalı: AI Copilot'un MCP uçuş komutları
        # (mcp_server.py FlightCommander) her komutta bu konuya yayın yapar.
        # Görülünce otonom rota izleme (900 m LOITER kuralı dahil) bırakılır —
        # aksi hâlde operatör uçağı başka yöne çevirdiğinde home'dan uzaklık
        # 900 m'yi aşar aşmaz bu düğüm uçağı LOITER'a zorlayıp komutu ezerdi.
        self.operator_override = False
        self.override_sub = self.create_subscription(
            String,
            "/siha/operator_override",
            self.override_callback,
            10
        )
        
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10
        )
        
        self.global_pos_pub = self.create_publisher(
            GeoPoseStamped,
            "/mavros/setpoint_position/global",
            10
        )
        
        self.current_pose = None
        self.current_gps = None
        self.start_pose = None
        self.target_pose = None
        self.target_gps = None
        self.route_targets = []
        self.route_target_index = 0
        self.target_name = None
        self.phase = "INIT"
        self.cruise_leg = 0
        
        self.get_logger().info("UAV Kontrolcü Düğümü Başlatıldı.")
        
        # Start the mission logic in a background thread to prevent deadlocking the executor
        self.mission_thread = threading.Thread(target=self.run_mission)
        self.mission_thread.daemon = True
        self.mission_thread.start()

    def pose_callback(self, msg):
        self.current_pose = msg

    def gps_callback(self, msg):
        self.current_gps = msg

    def override_callback(self, msg):
        # 'resume_route': operatör otonom rotaya dönmek istiyor — askı kalkar,
        # CRUISING döngüsü GUIDED + rota hedefini yeniden gönderip sürer.
        if msg.data == "resume_route":
            if self.operator_override:
                self.operator_override = False
                self.get_logger().info(
                    "Operatör otonom rotaya dönüş istedi; rota izleme "
                    "kaldığı yerden devam ediyor."
                )
            return
        # Diğer tüm mesajlar = manuel uçuş komutu. Her komutta yayın gelir;
        # yalnızca ilkinde loglayıp bayrağı kur.
        if not self.operator_override:
            self.operator_override = True
            self.get_logger().info(
                f"Operatör uçuş kontrolünü devraldı ({msg.data}); "
                "otonom rota izleme askıya alındı ('rotaya devam et' ile sürer)."
            )

    def wait_for_services(self):
        self.get_logger().info("MAVROS servislerinin bağlanması bekleniyor...")
        while rclpy.ok() and not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("SetMode servisi bekleniyor...")
        while rclpy.ok() and not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Arming servisi bekleniyor...")
        while rclpy.ok() and not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Takeoff servisi bekleniyor...")
        while rclpy.ok() and not self.wp_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("WaypointPush servisi bekleniyor...")
        while rclpy.ok() and not self.wp_clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("WaypointClear servisi bekleniyor...")
        while rclpy.ok() and not self.cmd_int_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("CommandInt servisi bekleniyor...")
        while rclpy.ok() and not self.cmd_long_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("CommandLong servisi bekleniyor...")
        if rclpy.ok():
            self.get_logger().info("Tüm servisler aktif.")

    def set_mode(self, mode_str):
        req = SetMode.Request()
        req.custom_mode = mode_str
        future = self.set_mode_client.call_async(req)
        # Block-wait for the future in this background thread without blocking the executor spin
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        if future.done():
            return future.result().mode_sent
        return False

    def arm_vehicle(self, arm_bool):
        req = CommandBool.Request()
        req.value = arm_bool
        future = self.arm_client.call_async(req)
        # Block-wait for the future in this background thread without blocking the executor spin
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        if future.done():
            if future.result().success or not arm_bool:
                return future.result().success
            self.get_logger().warn("Normal ARM reddedildi. SITL hızlı mod için force ARM deneniyor...")
            return self.force_arm_vehicle()
        return False

    def force_arm_vehicle(self):
        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0
        req.param1 = 1.0
        req.param2 = ARDUPILOT_FORCE_ARM_MAGIC

        future = self.cmd_long_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        if future.done():
            res = future.result()
            if res.success:
                self.get_logger().info("SITL force ARM komutu kabul edildi.")
            else:
                self.get_logger().warn(f"SITL force ARM reddedildi. MAV_RESULT={res.result}")
            return res.success
        return False

    def send_guided_target(self, lat, lon, alt):
        req = CommandInt.Request()
        req.frame = 3 # GLOBAL_RELATIVE_ALT
        req.command = 192 # MAV_CMD_DO_REPOSITION
        req.current = 2 # Guided "goto" target
        req.param1 = -1.0 # Speed (-1 = use default speed)
        req.param2 = 0.0 # Reposition flags
        req.param3 = 0.0 # Reserved
        req.param4 = float('nan') # Let ArduPlane align toward each route target
        req.x = int(lat * 1e7)
        req.y = int(lon * 1e7)
        req.z = float(alt)
        
        future = self.cmd_int_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        if future.done():
            res = future.result()
            self.get_logger().info(f"Target sent result: {res.success}")
            return res.success
        return False

    def send_route_target(self):
        # Aktif rota hedefini SADECE BİR KEZ (başarılı olana dek tekrar
        # deneyerek) gönderir. Sürekli yeniden gönderim navigasyonu sıfırlayıp
        # uçağın takılmasına yol açtığı için hedef başına tek gönderim yapılır.
        while rclpy.ok():
            if self.send_guided_target(self.target_lat, self.target_lon, self.target_alt):
                return True
            self.get_logger().warn("Rota hedefi gönderilemedi, tekrar deneniyor...")
            time.sleep(0.5)
        return False

    @staticmethod
    def distance_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
        lat1 = math.radians(lat1_deg)
        lon1 = math.radians(lon1_deg)
        lat2 = math.radians(lat2_deg)
        lon2 = math.radians(lon2_deg)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        return 6371000.0 * 2.0 * math.asin(math.sqrt(a))

    @staticmethod
    def bearing_deg(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
        lat1 = math.radians(lat1_deg)
        lat2 = math.radians(lat2_deg)
        dlon = math.radians(lon2_deg - lon1_deg)
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    @staticmethod
    def destination_point(lat_deg, lon_deg, bearing_deg, distance_m):
        radius_m = 6371000.0
        angular_distance = distance_m / radius_m
        bearing = math.radians(bearing_deg)
        lat1 = math.radians(lat_deg)
        lon1 = math.radians(lon_deg)
        lat2 = math.asin(
            math.sin(lat1) * math.cos(angular_distance)
            + math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing)
        )
        lon2 = lon1 + math.atan2(
            math.sin(bearing) * math.sin(angular_distance) * math.cos(lat1),
            math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2),
        )
        return math.degrees(lat2), math.degrees(lon2)

    def build_forward_route_targets(self, home_lat, home_lon):
        targets = []
        for name, distance_m in ROUTE_TARGET_SPECS:
            lat, lon = self.destination_point(
                home_lat,
                home_lon,
                STRAIGHT_ROUTE_HEADING_DEG,
                distance_m,
            )
            targets.append((name, lat, lon))
        return targets

    def run_mission(self):
        # 0. Wait for UAV position and GPS data
        last_info_time = 0
        while rclpy.ok() and (self.current_pose is None or self.current_gps is None):
            now = time.time()
            if now - last_info_time > 3.0:
                self.get_logger().info("Uçaktan pozisyon ve GPS bilgisi bekleniyor...")
                last_info_time = now
            time.sleep(0.2)

        if not rclpy.ok():
            return

        # 1. INIT: MAVROS Services
        self.wait_for_services()
        if not rclpy.ok():
            return

        # Kalkış (home) konumunu sabitle: kat edilen mesafe buradan ölçülür.
        self.home_lat = self.current_gps.latitude
        self.home_lon = self.current_gps.longitude
        self.route_targets = self.build_forward_route_targets(
            self.home_lat,
            self.home_lon,
        )
        self.get_logger().info(
            "Home konumuna göre düz hedef rotası oluşturuldu: "
            + ", ".join(f"{name}=({lat:.5f}, {lon:.5f})" for name, lat, lon in self.route_targets)
        )

        # 2. Upload Takeoff Mission and set to AUTO mode
        self.get_logger().info("Eski rota temizleniyor...")
        clear_req = WaypointClear.Request()
        future = self.wp_clear_client.call_async(clear_req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        self.get_logger().info("Otomatik pist kalkış misyonu yükleniyor...")
        wps = []
        # WP0: Home (Required)
        wp0 = Waypoint()
        wp0.frame = 3  # GLOBAL_RELATIVE_ALT
        wp0.command = 16  # NAV_WAYPOINT
        wp0.is_current = True
        wp0.autocontinue = True
        wp0.x_lat = self.current_gps.latitude
        wp0.y_long = self.current_gps.longitude
        wp0.z_alt = 0.0
        wps.append(wp0)

        # WP1: Takeoff
        wp1 = Waypoint()
        wp1.frame = 3
        wp1.command = 22  # NAV_TAKEOFF
        wp1.is_current = False
        wp1.autocontinue = True
        wp1.param1 = 15.0  # Minimum pitch angle
        wp1.x_lat = self.current_gps.latitude
        wp1.y_long = self.current_gps.longitude
        wp1.z_alt = CRUISE_ALTITUDE_M  # Kalkış irtifası (15 m)
        wps.append(wp1)

        # WP2: First target after takeoff. This starts the route over detected targets.
        wp2 = Waypoint()
        wp2.frame = 3
        wp2.command = 16  # NAV_WAYPOINT
        wp2.is_current = False
        wp2.autocontinue = True
        _, first_target_lat, first_target_lon = self.route_targets[0]
        wp2.x_lat = first_target_lat
        wp2.y_long = first_target_lon
        wp2.z_alt = CRUISE_ALTITUDE_M
        wps.append(wp2)

        push_req = WaypointPush.Request()
        push_req.start_index = 0
        push_req.waypoints = wps
        future = self.wp_push_client.call_async(push_req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        if future.done() and future.result().success:
            self.get_logger().info("Kalkış misyonu başarıyla yüklendi.")
        else:
            self.get_logger().error("Kalkış misyonu yüklenemedi! Görev sonlandırılıyor.")
            return

        # 3. Set flight mode to AUTO
        self.get_logger().info("Uçuş modu AUTO olarak ayarlanıyor...")
        while rclpy.ok():
            if self.set_mode("AUTO"):
                self.get_logger().info("Mod AUTO yapıldı.")
                self.phase = "ARMING"
                break
            else:
                self.get_logger().warn("AUTO moduna geçilemedi. Tekrar deneniyor...")
                time.sleep(1.0)

        # 4. ARMING: Motorları çalıştırıyoruz
        self.get_logger().info("Uçak motorları çalıştırılıyor (ARM)...")
        attempt = 0
        while rclpy.ok():
            if self.arm_vehicle(True):
                self.get_logger().info("Uçak başarıyla ARM edildi. Otomatik pist kalkışı başlıyor!")
                self.start_pose = self.current_pose
                self.phase = "CLIMBING"
                break
            else:
                attempt += 1
                if attempt % 5 == 1:
                    self.get_logger().warn("ARM başarısız. (Not: Simülasyon yeni başladıysa EKF/GPS/Sensör hizalaması 30-50 saniye sürebilir, lütfen iptal etmeden bekleyiniz...)")
                else:
                    self.get_logger().warn("ARM başarısız. Tekrar deneniyor...")
                time.sleep(1.0)

        # 5. CLIMBING: Hedef irtifaya (≈15 m) tırmanmayı bekle
        last_log_time = 0
        while rclpy.ok():
            altitude = self.current_pose.pose.position.z
            now = time.time()
            if now - last_log_time > 3.0:
                self.get_logger().info(f"Yükseklik: {altitude:.2f} m. Hedef irtifa bekleniyor...")
                last_log_time = now
            
            if altitude > CRUISE_ALTITUDE_M - 2.0:
                self.get_logger().info("Hedef irtifaya ulaşıldı. GUIDED moda geçiliyor...")
                while rclpy.ok():
                    if self.set_mode("GUIDED"):
                        # Düz uçuş için hedefi rota bitişinin ÇOK ötesine koy
                        # (aim-ahead). Böylece uçak ara noktalarda loiter'a
                        # girmeden 900 m boyunca dümdüz gider.
                        self.target_lat, self.target_lon = self.destination_point(
                            self.home_lat,
                            self.home_lon,
                            STRAIGHT_ROUTE_HEADING_DEG,
                            STRAIGHT_ROUTE_AIM_AHEAD_M,
                        )
                        self.target_alt = CRUISE_ALTITUDE_M
                        self.target_name = "Düz rota (ileri hedef)"
                        self.phase = "CRUISING"
                        self.get_logger().info(
                            f"Düz rota başlatıldı. Uçak {STRAIGHT_ROUTE_HEADING_DEG:.0f}° "
                            f"yönünde {CRUISE_ALTITUDE_M:.0f} m irtifada uçacak ve "
                            f"{STRAIGHT_ROUTE_TOTAL_M:.0f} m sonunda duracak."
                        )
                        break
                    else:
                        self.get_logger().warn("GUIDED moda geçilemedi! Tekrar deneniyor...")
                        time.sleep(1.0)
                break
            time.sleep(0.2)

        # 6. CRUISING: Uçak uzaktaki TEK hedefe doğru DÜMDÜZ uçar (hedef yalnızca
        # bir kez gönderilir). Home'dan kat edilen mesafe 900 m'ye ulaşınca
        # LOITER'a geçip durur. Yakın ara-hedefe "varıp daire çizme" sorunu
        # böylece tamamen ortadan kalkar. Yol üstündeki cisimlerin üzerinden
        # geçişi yalnızca bilgi amaçlı loglanır (navigasyonu etkilemez).
        last_log_time = 0
        passed_objects = set()
        # Rota hedefi gerektiğinde BİR kez gönderilir: görev başında ve her
        # operatör devralması sonrası 'resume_route' dönüşünde.
        target_needs_send = True
        while rclpy.ok() and self.phase == "CRUISING":
            # Operatör devraldıysa rota izleme ASKIDA bekler; döngü canlı kalır
            # ki 'resume_route' gelince kaldığı yerden sürebilsin. Askıda 900 m
            # kuralı da işlemez — operatörün komutu ezilmez.
            if self.operator_override:
                target_needs_send = True
                time.sleep(0.2)
                continue
            if target_needs_send:
                # Devralma dönüşünde uçak fly_forward sonrası LOITER'da olabilir;
                # GUIDED'a çekip rota hedefini yeniden gönder (görev başındaki
                # ilk gönderimde mod zaten GUIDED, çağrı zararsız).
                self.set_mode("GUIDED")
                self.send_route_target()
                target_needs_send = False
            now = time.time()
            # Kat edilen mesafe = rotanın KUZEY ekseni üzerindeki ilerleme
            # (izdüşüm; rota sabit 0° kuzey olduğu için enlem farkı yeterli).
            # Düz kuş-uçuşu uzaklık kullanılsaydı, operatör detourundan dönüşte
            # yana kaçıklık 900 m kuralını olduğundan erken tetikleyebilirdi.
            traveled = self.distance_m(
                self.home_lat,
                self.home_lon,
                self.current_gps.latitude,
                self.home_lon,
            )
            if self.current_gps.latitude < self.home_lat:
                traveled = 0.0

            # Yol üstündeki cisimlerin üzerinden geçişi bir kez logla.
            for name, tlat, tlon in self.route_targets:
                if name not in passed_objects:
                    d_obj = self.distance_m(
                        self.current_gps.latitude,
                        self.current_gps.longitude,
                        tlat,
                        tlon,
                    )
                    if d_obj < ROUTE_TARGET_REACHED_M:
                        passed_objects.add(name)
                        self.get_logger().info(
                            f"Cismin üzerinden geçildi: {name} (kat edilen ~{traveled:.0f} m)"
                        )

            if now - last_log_time > 2.0:
                self.get_logger().info(
                    f"Kat edilen mesafe: {traveled:.1f} / {STRAIGHT_ROUTE_TOTAL_M:.0f} m"
                )
                last_log_time = now

            if traveled >= STRAIGHT_ROUTE_TOTAL_M:
                self.get_logger().info(
                    f"{STRAIGHT_ROUTE_TOTAL_M:.0f} m düz rota tamamlandı. "
                    "LOITER moduna geçiliyor ve uçak bu noktada bekleyecek."
                )
                while rclpy.ok() and not self.set_mode("LOITER"):
                    self.get_logger().warn("LOITER moduna geçilemedi! Tekrar deneniyor...")
                    time.sleep(1.0)
                self.phase = "COMPLETE"
                break
            time.sleep(0.1)

        if self.phase == "COMPLETE":
            self.get_logger().info("Görev tamamlandı. Uçak 900 m sonunda LOITER ile bekliyor.")


def main(args=None):
    rclpy.init(args=args)
    node = UavController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Kontrolcü kapatılıyor...")
    except Exception as exc:
        # Kapanış yarışı: SIGINT'te spin ExternalShutdown yerine ham RCLError
        # fırlatabilir. Bağlam kapandıysa sessiz yut, yoksa yeniden fırlat.
        if rclpy.ok():
            raise
        node.get_logger().info(f"Kapanış sırasında yoksayıldı: {exc}")
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
