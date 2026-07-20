"""MCP (Model Context Protocol) Akış Durumu Takip Paneli.

GCS arayüzünde AI Copilot ve MCP sunucusu üzerinden tetiklenen her bir araç çağrısının
(Tool Calls) kendine özel dynamic mimari akış şemasını (Operatör -> LLM -> MCP Sunucu -> SİHA)
canlı ve adım adım üreten süreç takip widget'ı.
"""

import json
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QScrollArea, QFrame, QPushButton
)
from PySide6.QtCore import Qt, QTimer

# Araç isimlerini anlaşılır Türkçe açıklamalara ve simgelere eşleyen sözlük
TOOL_TITLE_MAP = {
    "get_telemetry": ("Anlık Telemetri Sorgula", "📡"),
    "get_telemetry_history": ("Geçmiş Telemetri Trendi", "📈"),
    "get_battery_status": ("Batarya Durumu Sorgula", "🔋"),
    "get_detections": ("Canlı Nesne Tespitleri", "🎯"),
    "get_detection_history": ("Tespit Geçmişi Günlüğü", "📜"),
    "get_vlm_scene": ("Görsel Sahne Analizi (VLM)", "👁"),
    "get_situation_summary": ("Genel Durum Özeti", "📊"),
    "change_altitude": ("İrtifa Komutu Gönder", "✈"),
    "change_speed": ("Hız Komutu Gönder", "⚡"),
    "turn_heading": ("Yön Komutu Gönder", "🧭"),
    "fly_forward": ("İleri Uçuş Komutu", "⏩"),
    "return_to_start": ("Başlangıç Noktasına Dön", "🏠"),
    "resume_route": ("Otonom Rotaya Dön", "🗺"),
    "start_flight": ("Uçuşa Geç / Kalkış", "🚀"),
    "generate_tactical_report": ("Taktik Rapor Derle", "📝"),
}


def _format_args_tr(args):
    """Girdi parametrelerini ham JSON yerine Türkçe ve okunabilir formatlar."""
    if not args or not isinstance(args, dict):
        return "<span style='color:#64748b; font-style:italic;'>Parametresiz Çağrı</span>"
    
    key_map = {
        "target_altitude": "Hedef İrtifa",
        "target_speed": "Hedef Hız",
        "heading_delta_deg": "Açı Değişimi",
        "distance_m": "Mesafe",
        "change_ms": "Hız Farkı",
        "query": "Arama Sorgusu",
        "limit": "Kayıt Sınırı",
    }
    
    items = []
    for k, v in args.items():
        label = key_map.get(k, k.replace("_", " ").title())
        unit = "m" if "altitude" in k or "distance" in k else ("°" if "heading" in k else "")
        items.append(f"{label}: <b style='color:#38bdf8;'>{v}{unit}</b>")
    
    return " &nbsp;|&nbsp; ".join(items)


def _format_result_tr(result):
    """İşlem sonucunu sade ve Türkçe metne çevirir."""
    if result is None:
        return "<span style='color:#64748b;'>Yanıt bekleniyor...</span>"
    if isinstance(result, str):
        return result
    if isinstance(result, dict):
        if "status" in result and isinstance(result["status"], str):
            status_text = result["status"]
            if "alt" in result:
                status_text += f" (Yeni İrtifa: {result['alt']}m)"
            elif "heading" in result:
                status_text += f" (Yeni Yön: {result['heading']}°)"
            return f"<span style='color:#34d399;'>{status_text}</span>"
        if "message" in result:
            return f"<span style='color:#34d399;'>{result['message']}</span>"
        
        parts = []
        for k, v in result.items():
            if k in ("ui_action", "ui_payload"):
                continue
            if isinstance(v, list):
                parts.append(f"{k}: <b>{len(v)} adet</b>")
            elif isinstance(v, dict):
                parts.append(f"{k}: <b>özet veri</b>")
            else:
                parts.append(f"{k}: <b>{v}</b>")
        out_str = " &nbsp;|&nbsp; ".join(parts)
        if len(out_str) > 160:
            out_str = out_str[:160] + "…"
        return out_str
    if isinstance(result, list):
        return f"<span style='color:#34d399;'>{len(result)} veri kaydı alındı</span>"
    return str(result)


def _get_siha_summary(tool_name, args, result):
    """SİHA düğümü için kısa durum özet metni üretir."""
    args = args or {}
    if tool_name == "change_altitude" and "target_altitude" in args:
        return f"{args['target_altitude']}m İrtifa"
    if tool_name == "change_speed" and "target_speed" in args:
        return f"{args['target_speed']} m/s Hız"
    if tool_name == "turn_heading" and "heading_delta_deg" in args:
        return f"{args['heading_delta_deg']}° Dönüş"
    if tool_name == "fly_forward" and "distance_m" in args:
        return f"{args['distance_m']}m İleri"
    if tool_name in ("get_telemetry", "get_telemetry_history"):
        return "Telemetri Verisi Alındı"
    if tool_name in ("get_detections", "get_detection_history"):
        return "Tespit Verisi Alındı"
    if tool_name == "get_vlm_scene":
        return "Sahne Özetlendi"
    if tool_name == "get_battery_status":
        return "Batarya Durumu Alındı"
    if tool_name == "generate_tactical_report":
        return "Rapor Derlendi"
    if tool_name == "return_to_start":
        return "Eve Dönüş Modu"
    if tool_name == "resume_route":
        return "Otonom Rota Aktif"
    if tool_name == "start_flight":
        return "Kalkış Başlatıldı"
    return "Tamamlandı"


class McpCallArchitectureFlowBar(QFrame):
    """Her çağrıya özel dinamik olarak oluşturulan MCP akış diyagramı çubuğu."""

    def __init__(self, tool_name, args=None, parent=None):
        super().__init__(parent)
        self.tool_name = tool_name
        self.args = args or {}
        self.setStyleSheet("""
            QFrame {
                background-color: #060b17;
                border: 1px solid rgba(56, 189, 248, 0.2);
                border-radius: 5px;
            }
        """)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        title_tr, icon = TOOL_TITLE_MAP.get(self.tool_name, (self.tool_name, "🔧"))

        self.lbl_op = QLabel("👤 OPERATÖR")
        self.lbl_op.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#94a3b8; background:none;")

        self.arr1 = QLabel("➔")
        self.arr1.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#06b6d4; background:none;")

        self.lbl_llm = QLabel("🧠 LLM MODEL")
        self.lbl_llm.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#38bdf8; background:none;")

        self.arr2 = QLabel("➔")
        self.arr2.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#06b6d4; background:none;")

        self.lbl_mcp = QLabel(f"⚙ MCP ({title_tr})")
        self.lbl_mcp.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#10b981; background:none;")

        self.arr3 = QLabel("➔")
        self.arr3.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#06b6d4; background:none;")

        self.lbl_siha = QLabel("🚁 SİHA")
        self.lbl_siha.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#a855f7; background:none;")

        layout.addWidget(self.lbl_op, alignment=Qt.AlignCenter)
        layout.addWidget(self.arr1, alignment=Qt.AlignCenter)
        layout.addWidget(self.lbl_llm, alignment=Qt.AlignCenter)
        layout.addWidget(self.arr2, alignment=Qt.AlignCenter)
        layout.addWidget(self.lbl_mcp, alignment=Qt.AlignCenter)
        layout.addWidget(self.arr3, alignment=Qt.AlignCenter)
        layout.addWidget(self.lbl_siha, alignment=Qt.AlignCenter)

    def update_flow_state(self, status, result=None):
        title_tr, icon = TOOL_TITLE_MAP.get(self.tool_name, (self.tool_name, "🔧"))

        if status == "waiting":
            self.lbl_mcp.setText(f"⚙ MCP ({title_tr})")
            self.lbl_mcp.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#94a3b8; background:none;")
            self.arr2.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#475569; background:none;")
            self.arr3.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#475569; background:none;")
            self.lbl_siha.setText("🚁 SİHA")
            self.lbl_siha.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#64748b; background:none;")
        elif status == "running":
            self.lbl_mcp.setText(f"⚙ İŞLENİYOR ({title_tr})")
            self.lbl_mcp.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#f59e0b; background:none;")
            self.arr2.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#f59e0b; font-weight:bold; background:none;")
            self.arr3.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#475569; background:none;")
            self.lbl_siha.setText("🚁 SİHA (İşleniyor...)")
            self.lbl_siha.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#94a3b8; background:none;")
        elif status == "success":
            siha_summary = _get_siha_summary(self.tool_name, self.args, result)
            self.lbl_mcp.setText(f"✓ MCP ({title_tr})")
            self.lbl_mcp.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#10b981; background:none;")
            self.arr2.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#10b981; font-weight:bold; background:none;")
            self.arr3.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#10b981; font-weight:bold; background:none;")
            self.lbl_siha.setText(f"🚁 SİHA ({siha_summary})")
            self.lbl_siha.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#a855f7; background:none;")
        elif status == "error":
            self.lbl_mcp.setText(f"✕ MCP ({title_tr})")
            self.lbl_mcp.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#ef4444; background:none;")
            self.arr2.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#ef4444; font-weight:bold; background:none;")
            self.arr3.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#ef4444; font-weight:bold; background:none;")
            self.lbl_siha.setText("🚁 SİHA (Başarısız)")
            self.lbl_siha.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; font-weight:bold; color:#ef4444; background:none;")


class McpStepProcessFlowWidget(QFrame):
    """Her çağrıya özel oluşturulan MCP Adım Kartı."""

    STATUS_CONFIG = {
        "waiting": {
            "icon": "⏳ BEKLENİYOR",
            "bg": "rgba(148, 163, 184, 0.1)",
            "border": "rgba(148, 163, 184, 0.3)",
            "text": "#94a3b8",
            "card_border": "#1e293b",
        },
        "running": {
            "icon": "⚙ İŞLENİYOR...",
            "bg": "rgba(245, 158, 11, 0.15)",
            "border": "rgba(245, 158, 11, 0.4)",
            "text": "#f59e0b",
            "card_border": "#f59e0b",
        },
        "success": {
            "icon": "✓ TAMAMLANDI",
            "bg": "rgba(16, 185, 129, 0.15)",
            "border": "rgba(16, 185, 129, 0.4)",
            "text": "#10b981",
            "card_border": "rgba(16, 185, 129, 0.25)",
        },
        "error": {
            "icon": "✕ BAŞARISIZ",
            "bg": "rgba(239, 68, 68, 0.15)",
            "border": "rgba(239, 68, 68, 0.4)",
            "text": "#ef4444",
            "card_border": "rgba(239, 68, 68, 0.4)",
        },
    }

    def __init__(self, step_id, tool_name, args=None, status="waiting", timestamp="", parent=None):
        super().__init__(parent)
        self.step_id = step_id
        self.tool_name = tool_name
        self.args = args or {}
        self.status = status
        self.timestamp = timestamp
        self.duration_ms = None
        self.pulse_phase = False

        self._setup_ui()
        self.update_status(status)

    def _setup_ui(self):
        self.setStyleSheet("""
            QFrame {
                background-color: #090e1a;
                border: 1px solid #1e293b;
                border-radius: 6px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(6)

        # 1. Üst Satır: Türkçe Araç İsmi + Zaman + Durum Rozeti
        header_layout = QHBoxLayout()
        header_layout.setSpacing(6)

        title_tr, icon = TOOL_TITLE_MAP.get(self.tool_name, (self.tool_name, "🔧"))
        tool_lbl = QLabel(f"{icon} <b style='color:#38bdf8; font-size:11px;'>{title_tr}</b> <span style='color:#64748b; font-size:9px;'>({self.tool_name})</span>")
        tool_lbl.setStyleSheet("font-family:'Roboto Mono'; background:none;")
        tool_lbl.setTextFormat(Qt.RichText)
        header_layout.addWidget(tool_lbl)

        header_layout.addStretch()

        self.time_lbl = QLabel(self.timestamp)
        self.time_lbl.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#64748b; background:none;")
        header_layout.addWidget(self.time_lbl)

        self.status_badge = QLabel(self.STATUS_CONFIG["waiting"]["icon"])
        self.status_badge.setStyleSheet("""
            font-family: 'Roboto Mono', monospace;
            font-size: 8px;
            font-weight: bold;
            padding: 2px 6px;
            border-radius: 3px;
        """)
        header_layout.addWidget(self.status_badge)

        layout.addLayout(header_layout)

        # 2. HER ÇAĞRIYA ÖZEL OLUŞTURULAN DİNAMİK MİMARİ AKIŞ ÇUBUĞU
        self.flow_bar = McpCallArchitectureFlowBar(self.tool_name, self.args)
        layout.addWidget(self.flow_bar)

        # 3. İçerik Kutusu: Girdi Parametreleri ve İşlem Sonucu
        self.details_frame = QFrame()
        self.details_frame.setStyleSheet("""
            background-color: #050811;
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 4px;
        """)
        details_lyt = QVBoxLayout(self.details_frame)
        details_lyt.setContentsMargins(8, 6, 8, 6)
        details_lyt.setSpacing(4)

        # Girdi Parametreleri
        args_formatted = _format_args_tr(self.args)
        self.lbl_args = QLabel(f"<span style='color:#94a3b8; font-size:9px;'>Girdi:</span> {args_formatted}")
        self.lbl_args.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#cbd5e1; background:none;")
        self.lbl_args.setTextFormat(Qt.RichText)
        details_lyt.addWidget(self.lbl_args)

        # Çıktı / İşlem Sonucu
        self.lbl_result = QLabel("<span style='color:#94a3b8; font-size:9px;'>Sonuç:</span> <span style='color:#64748b;'>Yanıt bekleniyor...</span>")
        self.lbl_result.setStyleSheet("font-family:'Roboto Mono'; font-size:9px; color:#cbd5e1; background:none;")
        self.lbl_result.setWordWrap(True)
        self.lbl_result.setTextFormat(Qt.RichText)
        details_lyt.addWidget(self.lbl_result)

        layout.addWidget(self.details_frame)

    def update_status(self, status, result=None, error=None, duration_ms=None):
        self.status = status
        if duration_ms is not None:
            self.duration_ms = duration_ms

        cfg = self.STATUS_CONFIG.get(status, self.STATUS_CONFIG["waiting"])
        
        dur_text = f" ({self.duration_ms} ms)" if self.duration_ms is not None and status == "success" else ""
        self.status_badge.setText(f"{cfg['icon']}{dur_text}")
        self.status_badge.setStyleSheet(f"""
            font-family: 'Roboto Mono', monospace;
            font-size: 8px;
            font-weight: bold;
            padding: 2px 6px;
            border-radius: 3px;
            color: {cfg['text']};
            background-color: {cfg['bg']};
            border: 1px solid {cfg['border']};
        """)

        self.setStyleSheet(f"""
            QFrame {{
                background-color: #090e1a;
                border: 1px solid {cfg['card_border']};
                border-radius: 6px;
            }}
        """)

        self.time_lbl.setText(self.timestamp)
        self.flow_bar.update_flow_state(status, result)

        if status == "waiting":
            self.lbl_result.setText("<span style='color:#94a3b8; font-size:9px;'>Sonuç:</span> <span style='color:#64748b;'>Sıraya alındı...</span>")
        elif status == "running":
            self.lbl_result.setText("<span style='color:#94a3b8; font-size:9px;'>Sonuç:</span> <span style='color:#f59e0b;'>Araç çalıştırılıyor, SİHA verisi işleniyor...</span>")
        elif status == "error":
            err_str = str(error) if error else "Bilinmeyen araç hatası"
            self.lbl_result.setText(f"<span style='color:#94a3b8; font-size:9px;'>Sonuç:</span> <span style='color:#ef4444;'>Hata: {err_str}</span>")
        elif status == "success":
            res_formatted = _format_result_tr(result)
            self.lbl_result.setText(f"<span style='color:#94a3b8; font-size:9px;'>Sonuç:</span> {res_formatted}")

    def pulse(self):
        if self.status != "running":
            return
        self.pulse_phase = not self.pulse_phase
        color = "#06b6d4" if self.pulse_phase else "#f59e0b"
        self.status_badge.setStyleSheet(f"""
            font-family: 'Roboto Mono', monospace;
            font-size: 8px;
            font-weight: bold;
            padding: 2px 6px;
            border-radius: 3px;
            color: {color};
            background-color: rgba(245, 158, 11, 0.2);
            border: 1px solid {color};
        """)


class McpWorkflowTrackerWidget(QFrame):
    """MCP Akış Durumu (MCP Workflow Tracker) ana widget'ı."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setProperty("class", "panel")
        self.cards_map = {}

        self._setup_ui()

        self.pulse_timer = QTimer(self)
        self.pulse_timer.setInterval(400)
        self.pulse_timer.timeout.connect(self._pulse_running_cards)
        self.pulse_timer.start()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Panel Başlık Çubuğu
        header_bar = QFrame()
        header_bar.setProperty("class", "panel-header")
        header_bar.setFixedHeight(28)
        header_layout = QHBoxLayout(header_bar)
        header_layout.setContentsMargins(10, 0, 10, 0)

        title_lbl = QLabel("MCP SÜREÇ & ARAÇ AKIŞI (CANLI)")
        title_lbl.setProperty("class", "title-label")
        title_lbl.setStyleSheet("color: #38bdf8; font-weight: bold;")
        header_layout.addWidget(title_lbl)

        self.count_badge = QLabel("0 İŞLEM")
        self.count_badge.setStyleSheet("""
            background-color: rgba(56, 189, 248, 0.1);
            color: #38bdf8;
            border: 1px solid rgba(56, 189, 248, 0.3);
            border-radius: 3px;
            padding: 1px 5px;
            font-size: 8px;
            font-weight: bold;
            font-family: 'Roboto Mono';
        """)
        header_layout.addWidget(self.count_badge)

        header_layout.addStretch()

        clear_btn = QPushButton("Temizle")
        clear_btn.setStyleSheet("""
            background: transparent;
            color: #64748b;
            border: none;
            font-size: 9px;
            font-family: 'Roboto Mono';
            padding: 0 4px;
        """)
        clear_btn.setCursor(Qt.PointingHandCursor)
        clear_btn.clicked.connect(self.clear_steps)
        header_layout.addWidget(clear_btn)

        main_layout.addWidget(header_bar)

        # Scroll Area
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: #060910;
            }
        """)

        self.cards_container = QWidget()
        self.cards_layout = QVBoxLayout(self.cards_container)
        self.cards_layout.setContentsMargins(8, 8, 8, 8)
        self.cards_layout.setSpacing(6)
        self.cards_layout.setAlignment(Qt.AlignTop)

        # Bekleme Açıklaması
        self.placeholder_lbl = QLabel(
            "MCP araç çağrı akışı bekleniyor...\n\n"
            "AI Copilot veya araç butonları kullanıldığında tetiklenen\n"
            "her çağrıya özel LLM ➔ MCP ➔ SİHA akış şeması burada canlı görüntülenecektir."
        )
        self.placeholder_lbl.setAlignment(Qt.AlignCenter)
        self.placeholder_lbl.setStyleSheet("""
            font-family: 'Roboto Mono', monospace;
            font-size: 10px;
            color: #475569;
            padding: 20px 10px;
        """)
        self.cards_layout.addWidget(self.placeholder_lbl)

        self.scroll_area.setWidget(self.cards_container)
        main_layout.addWidget(self.scroll_area)

    def handle_tool_event(self, event_data):
        """GeminiChatThread veya MCP client'tan gelen sinyal verisiyle adımı ekler/günceller."""
        step_id = event_data.get("id")
        if not step_id:
            return

        name = event_data.get("name", "Bilinmeyen Araç")
        args = event_data.get("args")
        status = event_data.get("status", "running")
        timestamp = event_data.get("timestamp", "")
        duration_ms = event_data.get("duration_ms")
        result = event_data.get("result")
        error = event_data.get("error")

        if self.placeholder_lbl is not None:
            self.cards_layout.removeWidget(self.placeholder_lbl)
            self.placeholder_lbl.deleteLater()
            self.placeholder_lbl = None

        if step_id in self.cards_map:
            card = self.cards_map[step_id]
            card.update_status(status, result=result, error=error, duration_ms=duration_ms)
        else:
            card = McpStepProcessFlowWidget(
                step_id=step_id,
                tool_name=name,
                args=args,
                status=status,
                timestamp=timestamp
            )
            if status in ("success", "error"):
                card.update_status(status, result=result, error=error, duration_ms=duration_ms)
            self.cards_map[step_id] = card
            self.cards_layout.addWidget(card)

        total = len(self.cards_map)
        self.count_badge.setText(f"{total} İŞLEM")

        QTimer.singleShot(50, lambda: self.scroll_area.verticalScrollBar().setValue(
            self.scroll_area.verticalScrollBar().maximum()
        ))

    def clear_steps(self):
        for card in self.cards_map.values():
            card.deleteLater()
        self.cards_map.clear()
        self.count_badge.setText("0 İŞLEM")

        if self.placeholder_lbl is None:
            self.placeholder_lbl = QLabel(
                "MCP araç çağrı akışı bekleniyor...\n\n"
                "AI Copilot veya araç butonları kullanıldığında tetiklenen\n"
                "her çağrıya özel LLM ➔ MCP ➔ SİHA akış şeması burada canlı görüntülenecektir."
            )
            self.placeholder_lbl.setAlignment(Qt.AlignCenter)
            self.placeholder_lbl.setStyleSheet("""
                font-family: 'Roboto Mono', monospace;
                font-size: 10px;
                color: #475569;
                padding: 20px 10px;
            """)
            self.cards_layout.addWidget(self.placeholder_lbl)

    def _pulse_running_cards(self):
        for card in self.cards_map.values():
            if card.status == "running":
                card.pulse()
