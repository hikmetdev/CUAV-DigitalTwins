
document.addEventListener("DOMContentLoaded", () => {
    lucide.createIcons();

    // ----------------------------------------------------
    // 1. STATE & GLOBAL CONFIGURATION
    // ----------------------------------------------------
    const STRAIGHT_ROUTE_HEADING_DEG = 90;
    const ROUTE_STEP_DEG = 0.00004;
    const ROUTE_REACHED_DEG = 0.00006;
    const START_LAT = 39.920782;
    const START_LON = 32.854115;
    const TARGET_DETECTION_SPECS = [
        { id: "T-01", type: "Araç", confidence: 0.93, time: "12:30:12", distanceM: 250 },
        { id: "T-02", type: "Kişi", confidence: 0.91, time: "12:32:45", distanceM: 500 },
        { id: "T-03", type: "Araç", confidence: 0.88, time: "12:34:10", distanceM: 750 },
    ];

    function destinationPoint(latDeg, lonDeg, bearingDeg, distanceM) {
        const radiusM = 6371000;
        const angularDistance = distanceM / radiusM;
        const bearing = (bearingDeg * Math.PI) / 180;
        const lat1 = (latDeg * Math.PI) / 180;
        const lon1 = (lonDeg * Math.PI) / 180;
        const lat2 = Math.asin(
            Math.sin(lat1) * Math.cos(angularDistance) +
            Math.cos(lat1) * Math.sin(angularDistance) * Math.cos(bearing)
        );
        const lon2 = lon1 + Math.atan2(
            Math.sin(bearing) * Math.sin(angularDistance) * Math.cos(lat1),
            Math.cos(angularDistance) - Math.sin(lat1) * Math.sin(lat2)
        );
        return { lat: (lat2 * 180) / Math.PI, lon: (lon2 * 180) / Math.PI };
    }

    const targetDetections = TARGET_DETECTION_SPECS.map(det => ({
        ...det,
        ...destinationPoint(START_LAT, START_LON, STRAIGHT_ROUTE_HEADING_DEG, det.distanceM),
        marker: null
    }));

    const state = {
        telemetry: {
            alt: 84.2,       // meters
            speed: 12.8,     // m/s
            battery: 87,     // percentage
            voltage: 22.8,   // V
            mode: "AUTO.LOITER",
            lat: START_LAT,  // Ankara coordinates (SITL starting position)
            lon: START_LON,
            heading: STRAIGHT_ROUTE_HEADING_DEG,    // degrees (yaw)
            fps: 28.4,
            latency: 82,     // ms
        },
        pathIndex: 1,
        waypoints: [
            { lat: START_LAT, lon: START_LON, name: "START (GCS)" },
            ...targetDetections.map(det => ({ lat: det.lat, lon: det.lon, name: det.id }))
        ],
        detections: targetDetections.map(det => ({ ...det })),
        timelineEvents: [
            { time: "12:25:50", type: "command", desc: "Uçuş Öncesi Kontroller OK" },
            { time: "12:26:30", type: "waypoint", desc: "Kalkış Başarılı (ALT: 10m)" },
            { time: "12:28:15", type: "command", desc: "Otopilot Modu: AUTO.LOITER" },
            { time: "12:30:12", type: "vehicle", desc: "YOLO: Şüpheli Araç (T-01)" },
            { time: "12:32:45", type: "person", desc: "YOLO: 1 Yaya Sınır İhlali (T-02)" },
            { time: "12:34:10", type: "vehicle", desc: "YOLO: Araç Tespit Edildi (T-03)" },
        ],
        vlmSummaries: [
            "Saat 14:35'te, İHA 120m irtifadayken, [39.92, 32.85] koordinatında bir adet 'blue cargo box' %94 güvenle tespit edildi.",
            "Görüş alanında 2 araç ve 1 kişi bulunuyor. Çevre genel olarak sakin, hedefler takip altında tutuluyor.",
            "Tesis sınırı yakınındaki yaya güneybatı yönünde ilerliyor. Araçlarda hareket gözlemlenmedi.",
            "Tüm sistemler kararlı durumda. Kameralar hedeflere kilitlendi. LLM karar destek mekanizması aktif."
        ],
        vlmIndex: 0,
        logs: [],
        map: null,
        uavMarker: null,
        uavPathPolyline: null,
        isLogsOpen: false,
    };

    // ----------------------------------------------------
    // 2. SYSTEM CLOCK
    // ----------------------------------------------------
    function updateClock() {
        const now = new Date();
        const timeString = now.toTimeString().split(' ')[0];
        document.getElementById("system-time").innerText = timeString;
    }
    setInterval(updateClock, 1000);
    updateClock();

    // ----------------------------------------------------
    // 3. COLLAPSIBLE LOGS PANEL
    // ----------------------------------------------------
    const bottomTerminal = document.getElementById("bottom-terminal");
    const toggleLogsBtn = document.getElementById("toggle-logs-btn");
    const closeLogsBtn = document.getElementById("close-logs-btn");
    const logOutput = document.getElementById("log-output");

    function toggleLogs(open) {
        state.isLogsOpen = open;
        if (open) {
            bottomTerminal.style.height = "192px"; // 48rem/12rem equivalents
            toggleLogsBtn.classList.add("bg-brand-accent/20", "text-brand-accent", "border-brand-accent/60");
            // Auto scroll logs
            setTimeout(() => { logOutput.scrollTop = logOutput.scrollHeight; }, 310);
        } else {
            bottomTerminal.style.height = "0";
            toggleLogsBtn.classList.remove("bg-brand-accent/20", "text-brand-accent", "border-brand-accent/60");
        }
    }

    toggleLogsBtn.addEventListener("click", () => toggleLogs(!state.isLogsOpen));
    closeLogsBtn.addEventListener("click", () => toggleLogs(false));

    // Simulated Log generator
    const logTypes = ["INFO", "DEBUG", "WARN", "NAV"];
    const logDetails = [
        "MAVLink Heartbeat received from Component 1 (ID: 1)",
        "GPS lock status: 3D Fix, Sats: 15, HDOP: 0.85",
        "IMU calibration drift check: OK, Roll: 0.12, Pitch: -0.04",
        "EKF status: OK, Velocity variance check < 0.05",
        "YOLOv8 target recognition frame update (14ms inference)",
        "MAVSDK action call: guided_mode_tracker update executed",
        "Battery cell health: 3.82V, 3.81V, 3.82V, 3.81V, 3.82V, 3.81V",
        "VLM prompt generated. Scene token length: 242 tokens"
    ];

    function addLogMessage(message, type = "INFO") {
        const now = new Date();
        const timeStr = now.toTimeString().split(' ')[0] + "." + String(now.getMilliseconds()).padStart(3, '0');

        let colorClass = "text-emerald-400";
        if (type === "WARN") colorClass = "text-amber-400 font-semibold";
        if (type === "DEBUG") colorClass = "text-slate-400";
        if (type === "NAV") colorClass = "text-cyan-400";

        const logDiv = document.createElement("div");
        logDiv.innerHTML = `<span class="text-slate-500">[${timeStr}]</span> <span class="${colorClass}">[${type}]</span> ${message}`;
        logOutput.appendChild(logDiv);

        // Keep logs capped at 100
        while (logOutput.children.length > 100) {
            logOutput.removeChild(logOutput.firstChild);
        }

        if (state.isLogsOpen) {
            logOutput.scrollTop = logOutput.scrollHeight;
        }
    }

    // Populate initial logs
    for (let i = 0; i < 15; i++) {
        const type = logTypes[Math.floor(Math.random() * logTypes.length)];
        const msg = logDetails[Math.floor(Math.random() * logDetails.length)];
        addLogMessage(msg, type);
    }

    // Generate logs in intervals
    setInterval(() => {
        const type = logTypes[Math.floor(Math.random() * logTypes.length)];
        const msg = logDetails[Math.floor(Math.random() * logDetails.length)];
        addLogMessage(msg, type);
    }, 2500);


    // ----------------------------------------------------
    // 4. MAP: LEAFLET.JS IMPLEMENTATION
    // ----------------------------------------------------
    function initMap() {
        // Initializing map centered at Ankara, Turkey
        state.map = L.map('tactical-map', {
            zoomControl: false,
            attributionControl: false
        }).setView([state.telemetry.lat, state.telemetry.lon], 16);

        // CartoDB Dark Matter tiles (Perfect defense UI aesthetic)
        L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
            maxZoom: 20
        }).addTo(state.map);

        // Add custom Zoom Control at top-right
        L.control.zoom({ position: 'topright' }).addTo(state.map);

        // Custom icons
        const uavIcon = L.divIcon({
            className: 'uav-marker-icon',
            html: '<div class="uav-marker-glow" id="map-uav-marker"></div>',
            iconSize: [30, 30],
            iconAnchor: [15, 15]
        });

        // UAV Position Marker
        state.uavMarker = L.marker([state.telemetry.lat, state.telemetry.lon], { icon: uavIcon })
            .addTo(state.map)
            .bindPopup("<b>İHA (AD-GCS-01)</b><br>Mod: AUTO.LOITER<br>ALT: 84.2m<br>SITL Aktif");

        // Flight Path Polyline
        const pathCoords = state.waypoints.map(wp => [wp.lat, wp.lon]);
        state.uavPathPolyline = L.polyline(pathCoords, {
            color: '#06b6d4',
            weight: 2,
            opacity: 0.6,
            dashArray: '5, 5'
        }).addTo(state.map);

        // Render Waypoint Markers
        state.waypoints.forEach((wp, idx) => {
            const wpIcon = L.divIcon({
                className: 'waypoint-marker-icon',
                html: '<div class="waypoint-marker-glow"></div>',
                iconSize: [14, 14],
                iconAnchor: [7, 7]
            });

            L.marker([wp.lat, wp.lon], { icon: wpIcon })
                .addTo(state.map)
                .bindPopup(`<b>${wp.name}</b><br>Sıra: ${idx + 1}`);
        });

        // Render Initial Target Detections
        updateTargetMarkersOnMap();
    }

    function updateTargetMarkersOnMap() {
        state.detections.forEach(det => {
            if (det.marker) {
                state.map.removeLayer(det.marker);
            }

            const targetIcon = L.divIcon({
                className: 'target-marker-icon',
                html: '<div class="target-marker-glow"></div>',
                iconSize: [24, 24],
                iconAnchor: [12, 12]
            });

            det.marker = L.marker([det.lat, det.lon], { icon: targetIcon })
                .addTo(state.map)
                .bindPopup(`<b>HEDEF TESPİTİ (${det.id})</b><br>Nesne: ${det.type}<br>Güven: %${(det.confidence * 100).toFixed(0)}<br>GPS: ${det.lat.toFixed(5)}, ${det.lon.toFixed(5)}`);
        });
    }

    // Map button actions
    document.getElementById("map-recenter-btn").addEventListener("click", () => {
        if (state.map) {
            state.map.setView([state.telemetry.lat, state.telemetry.lon], 16);
            addLogMessage("Harita İHA konumuna merkezlendi.", "NAV");
        }
    });

    document.getElementById("map-clear-path-btn").addEventListener("click", () => {
        if (state.uavPathPolyline) {
            state.map.removeLayer(state.uavPathPolyline);
            addLogMessage("Harita üzerindeki uçuş rotası gizlendi.", "NAV");
        }
    });

    initMap();


    // ----------------------------------------------------
    // 5. CAMERA & YOLO INFOGRAPHIC CANVAS
    // ----------------------------------------------------
    const canvas = document.getElementById("camera-canvas");
    const ctx = canvas.getContext("2d");

    // Canvas size adjustment
    function resizeCanvas() {
        const rect = canvas.parentElement.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;
    }
    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();

    // Drone aerial simulator view state
    const camSim = {
        gridOffsetX: 0,
        gridOffsetY: 0,
        objects: [
            { id: "T-01", label: "car", x: 0.35, y: 0.40, w: 50, h: 30, conf: 0.93, color: "#f59e0b", speedX: -0.0002, speedY: 0.0001 },
            { id: "T-02", label: "person", x: 0.55, y: 0.65, w: 20, h: 20, conf: 0.91, color: "#f59e0b", speedX: 0.0001, speedY: -0.0003 },
            { id: "T-03", label: "car", x: 0.20, y: 0.25, w: 45, h: 28, conf: 0.88, color: "#f59e0b", speedX: 0.00005, speedY: 0.00008 }
        ]
    };

    function drawCameraSimulation(timestamp) {
        // Draw deep green fields/aerial styling
        ctx.fillStyle = "#0c160f"; // Very dark forest/ground green
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Draw moving grid lines (to simulate drone flying forward)
        ctx.strokeStyle = "rgba(16, 185, 129, 0.05)";
        ctx.lineWidth = 1;

        // Update offsets based on heading and speed
        const speedFactor = state.telemetry.speed * 0.05;
        const rad = (state.telemetry.heading * Math.PI) / 180;
        camSim.gridOffsetX += Math.sin(rad) * speedFactor;
        camSim.gridOffsetY += Math.cos(rad) * speedFactor;

        const gridSize = 60;
        const startX = (camSim.gridOffsetX % gridSize) - gridSize;
        const startY = (camSim.gridOffsetY % gridSize) - gridSize;

        for (let x = startX; x < canvas.width + gridSize; x += gridSize) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, canvas.height);
            ctx.stroke();
        }

        for (let y = startY; y < canvas.height + gridSize; y += gridSize) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(canvas.width, y);
            ctx.stroke();
        }

        // Draw stylized thermal-like topographic contours (subtle vector style)
        ctx.strokeStyle = "rgba(16, 185, 129, 0.02)";
        ctx.lineWidth = 1.5;
        for (let r = 100; r < Math.max(canvas.width, canvas.height); r += 150) {
            ctx.beginPath();
            ctx.arc(canvas.width / 2 + Math.sin(timestamp * 0.0001) * 30, canvas.height / 2 + Math.cos(timestamp * 0.0001) * 20, r, 0, Math.PI * 2);
            ctx.stroke();
        }

        // Draw Static Map Objects (Mock structures on ground)
        ctx.strokeStyle = "rgba(6, 182, 212, 0.1)";
        ctx.strokeRect(canvas.width * 0.1 + camSim.gridOffsetX % canvas.width, canvas.height * 0.2 + camSim.gridOffsetY % canvas.height, 100, 80);
        ctx.strokeRect(canvas.width * 0.7 + camSim.gridOffsetX % canvas.width, canvas.height * 0.6 + camSim.gridOffsetY % canvas.height, 150, 100);

        // Update and draw targets with bounding boxes
        camSim.objects.forEach(obj => {
            // Update position
            obj.x += obj.speedX;
            obj.y += obj.speedY;

            // Keep within bounds
            if (obj.x < 0.1 || obj.x > 0.9) obj.speedX *= -1;
            if (obj.y < 0.1 || obj.y > 0.9) obj.speedY *= -1;

            const drawX = obj.x * canvas.width;
            const drawY = obj.y * canvas.height;

            // Draw bounding box
            ctx.strokeStyle = obj.color;
            ctx.lineWidth = 1.5;

            // Draw corner markers for YOLO bbox look
            const cornerSize = 8;
            ctx.beginPath();
            // Top Left
            ctx.moveTo(drawX - obj.w / 2, drawY - obj.h / 2 + cornerSize);
            ctx.lineTo(drawX - obj.w / 2, drawY - obj.h / 2);
            ctx.lineTo(drawX - obj.w / 2 + cornerSize, drawY - obj.h / 2);
            // Top Right
            ctx.moveTo(drawX + obj.w / 2 - cornerSize, drawY - obj.h / 2);
            ctx.lineTo(drawX + obj.w / 2, drawY - obj.h / 2);
            ctx.lineTo(drawX + obj.w / 2, drawY - obj.h / 2 + cornerSize);
            // Bottom Right
            ctx.moveTo(drawX + obj.w / 2, drawY + obj.h / 2 - cornerSize);
            ctx.lineTo(drawX + obj.w / 2, drawY + obj.h / 2);
            ctx.lineTo(drawX + obj.w / 2 - cornerSize, drawY + obj.h / 2);
            // Bottom Left
            ctx.moveTo(drawX - obj.w / 2 + cornerSize, drawY + obj.h / 2);
            ctx.lineTo(drawX - obj.w / 2, drawY + obj.h / 2);
            ctx.lineTo(drawX - obj.w / 2, drawY + obj.h / 2 - cornerSize);
            ctx.stroke();

            // Box Fill glow
            ctx.fillStyle = "rgba(245, 158, 11, 0.05)";
            ctx.fillRect(drawX - obj.w / 2, drawY - obj.h / 2, obj.w, obj.h);

            // Bounding box labels
            ctx.fillStyle = obj.color;
            ctx.font = "bold 9px 'Roboto Mono'";
            ctx.fillText(`${obj.label.toUpperCase()} %${(obj.conf * 100).toFixed(0)}`, drawX - obj.w / 2, drawY - obj.h / 2 - 4);

            // Reticle pointer/center
            ctx.beginPath();
            ctx.arc(drawX, drawY, 2, 0, Math.PI * 2);
            ctx.fill();
        });

        // Render Artificial HUD Overlay
        drawHUDOverlay();

        requestAnimationFrame(drawCameraSimulation);
    }

    function drawHUDOverlay() {
        ctx.strokeStyle = "rgba(16, 185, 129, 0.25)";
        ctx.lineWidth = 1;

        // Draw HUD Corner Brackets
        const len = 20;
        const pad = 15;
        // TL
        ctx.beginPath(); ctx.moveTo(pad, pad + len); ctx.lineTo(pad, pad); ctx.lineTo(pad + len, pad); ctx.stroke();
        // TR
        ctx.beginPath(); ctx.moveTo(canvas.width - pad, pad + len); ctx.lineTo(canvas.width - pad, pad); ctx.lineTo(canvas.width - pad - len, pad); ctx.stroke();
        // BL
        ctx.beginPath(); ctx.moveTo(pad, canvas.height - pad - len); ctx.lineTo(pad, canvas.height - pad); ctx.lineTo(pad + len, canvas.height - pad); ctx.stroke();
        // BR
        ctx.beginPath(); ctx.moveTo(canvas.width - pad, canvas.height - pad - len); ctx.lineTo(canvas.width - pad, canvas.height - pad); ctx.lineTo(canvas.width - pad - len, canvas.height - pad); ctx.stroke();

        // Target Finder Center Lines
        ctx.strokeStyle = "rgba(16, 185, 129, 0.15)";
        ctx.beginPath(); ctx.moveTo(canvas.width / 2 - 80, canvas.height / 2); ctx.lineTo(canvas.width / 2 - 20, canvas.height / 2); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(canvas.width / 2 + 20, canvas.height / 2); ctx.lineTo(canvas.width / 2 + 80, canvas.height / 2); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(canvas.width / 2, canvas.height / 2 - 80); ctx.lineTo(canvas.width / 2, canvas.height / 2 - 20); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(canvas.width / 2, canvas.height / 2 + 20); ctx.lineTo(canvas.width / 2, canvas.height / 2 + 80); ctx.stroke();
    }

    requestAnimationFrame(drawCameraSimulation);


    // ----------------------------------------------------
    // 6. TELEMETRY STREAM SIMULATION
    // ----------------------------------------------------
    function runTelemetrySimulation() {
        setInterval(() => {
            // Altitude subtle drift
            state.telemetry.alt = Math.max(78.5, Math.min(95.0, state.telemetry.alt + (Math.random() - 0.5) * 0.4));

            // Speed subtle drift
            state.telemetry.speed = Math.max(10.0, Math.min(15.5, state.telemetry.speed + (Math.random() - 0.5) * 0.2));

            state.telemetry.heading = STRAIGHT_ROUTE_HEADING_DEG;

            // Battery depletion
            if (state.telemetry.battery > 5) {
                state.telemetry.battery -= 0.05; // Deplete slowly
                state.telemetry.voltage = (18.0 + (state.telemetry.battery / 100) * 5.0).toFixed(1);
            } else {
                state.telemetry.battery = 87; // reset mock telemetry after battery drains
                addLogMessage("Simülatör Batarya Değişimi: %87 seviyesine yükseltildi.", "WARN");
            }

            let dLat;
            let dLon;
            let targetWP;
            if (state.pathIndex < state.waypoints.length) {
                targetWP = state.waypoints[state.pathIndex];
                dLat = targetWP.lat - state.telemetry.lat;
                dLon = targetWP.lon - state.telemetry.lon;
            } else {
                const prevWP = state.waypoints[state.waypoints.length - 2];
                targetWP = state.waypoints[state.waypoints.length - 1];
                dLat = targetWP.lat - prevWP.lat;
                dLon = targetWP.lon - prevWP.lon;
            }

            const distance = Math.sqrt(dLat * dLat + dLon * dLon);
            if (distance > 0) {
                const targetAngle = Math.floor((Math.atan2(dLon, dLat) * 180) / Math.PI);
                state.telemetry.heading = (targetAngle + 360) % 360;
            }

            if (state.pathIndex < state.waypoints.length && distance <= ROUTE_REACHED_DEG) {
                state.telemetry.lat = targetWP.lat;
                state.telemetry.lon = targetWP.lon;
                addLogMessage(`Hedef üzerinden geçildi: ${targetWP.name}`, "NAV");
                addTimelineEvent("waypoint", `Hedef geçildi: ${targetWP.name}`);
                state.pathIndex++;
            } else if (distance > 0) {
                const step = Math.min(ROUTE_STEP_DEG, distance);
                state.telemetry.lat += (dLat / distance) * step;
                state.telemetry.lon += (dLon / distance) * step;
            }

            // Update UI components
            updateTelemetryUI();

        }, 1000);
    }

    function updateTelemetryUI() {
        document.getElementById("telemetry-alt").innerText = state.telemetry.alt.toFixed(1);
        document.getElementById("telemetry-speed").innerText = state.telemetry.speed.toFixed(1);

        const roundedBat = Math.round(state.telemetry.battery);
        document.getElementById("telemetry-battery").innerText = roundedBat;
        document.getElementById("battery-voltage").innerText = `${state.telemetry.voltage} V`;

        const batBar = document.getElementById("battery-bar");
        batBar.style.width = `${roundedBat}%`;

        // Update colors based on battery status
        const batteryIcon = document.getElementById("battery-icon");
        if (roundedBat < 20) {
            batBar.className = "bg-brand-alert h-full shadow-[0_0_8px_#ef4444]";
            document.getElementById("telemetry-battery").className = "text-xl font-bold text-brand-alert";
            batteryIcon.className = "w-5 h-5 text-brand-alert animate-bounce";
        } else if (roundedBat < 50) {
            batBar.className = "bg-brand-warn h-full shadow-[0_0_8px_#f59e0b]";
            document.getElementById("telemetry-battery").className = "text-xl font-bold text-brand-warn";
            batteryIcon.className = "w-5 h-5 text-brand-warn";
        } else {
            batBar.className = "bg-brand-accent h-full shadow-[0_0_8px_#10b981]";
            document.getElementById("telemetry-battery").className = "text-xl font-bold text-brand-accent";
            batteryIcon.className = "w-5 h-5 text-brand-accent";
        }

        document.getElementById("telemetry-heading").innerText = state.telemetry.heading;
        document.getElementById("compass-pointer").style.transform = `rotate(${state.telemetry.heading}deg)`;
        document.getElementById("hud-heading").innerText = `${state.telemetry.heading}°`;

        // Update coordinate DOM elements
        const latElements = document.getElementsByClassName("gps-lat");
        const lonElements = document.getElementsByClassName("gps-lon");

        for (let el of latElements) el.innerText = state.telemetry.lat.toFixed(6);
        for (let el of lonElements) el.innerText = state.telemetry.lon.toFixed(6);

        // Update Leaflet Marker position
        if (state.uavMarker) {
            state.uavMarker.setLatLng([state.telemetry.lat, state.telemetry.lon]);
            state.uavMarker.setPopupContent(`<b>İHA (AD-GCS-01)</b><br>Mod: ${state.telemetry.mode}<br>ALT: ${state.telemetry.alt.toFixed(1)}m<br>HIZ: ${state.telemetry.speed.toFixed(1)} m/s`);
        }
    }

    runTelemetrySimulation();


    // ----------------------------------------------------
    // 7. ACTIVE DETECTIONS TABLE
    // ----------------------------------------------------
    function renderDetectionsTable() {
        const tbody = document.getElementById("detections-table-body");
        tbody.innerHTML = "";

        state.detections.forEach(det => {
            const tr = document.createElement("tr");
            tr.className = "hover:bg-slate-800/40 cursor-pointer transition-colors duration-150 border-b border-brand-border/10";
            tr.innerHTML = `
                <td class="py-2.5 px-3 font-bold text-slate-400">${det.id}</td>
                <td class="py-2.5 px-2">
                    <span class="flex items-center space-x-1.5">
                        <span class="w-1.5 h-1.5 rounded-full ${det.type === 'Araç' ? 'bg-brand-warn shadow-[0_0_4px_#f59e0b]' : 'bg-orange-500 shadow-[0_0_4px_#f97316]'}"></span>
                        <span>${det.type}</span>
                    </span>
                </td>
                <td class="py-2.5 px-2 font-bold text-brand-cyan">${(det.confidence * 100).toFixed(0)}%</td>
                <td class="py-2.5 px-2 text-slate-500">${det.time}</td>
                <td class="py-2.5 px-3 text-right text-slate-300 font-mono text-[10px] hover:text-brand-accent">${det.lat.toFixed(5)}, ${det.lon.toFixed(5)}</td>
            `;

            // Click table row -> center map on target
            tr.addEventListener("click", () => {
                if (state.map) {
                    state.map.setView([det.lat, det.lon], 18);
                    det.marker.openPopup();
                    addLogMessage(`Harita ${det.id} hedef konumuna odaklandı.`, "NAV");
                }
            });

            tbody.appendChild(tr);
        });
    }

    renderDetectionsTable();


    // ----------------------------------------------------
    // 8. TIMELINE & VLM SUMMARY UPDATES
    // ----------------------------------------------------
    function renderTimeline() {
        const container = document.getElementById("timeline-points-container");
        container.innerHTML = "";

        const count = state.timelineEvents.length;
        state.timelineEvents.forEach((ev, idx) => {
            const leftPercent = 5 + (idx / (count - 1)) * 90; // Space points nicely

            // Choose color based on type
            let color = "bg-brand-accent";
            if (ev.type === "vehicle") color = "bg-brand-alert";
            if (ev.type === "person") color = "bg-brand-warn";
            if (ev.type === "waypoint") color = "bg-brand-cyan";

            const point = document.createElement("div");
            point.className = `absolute timeline-dot w-3 h-3 ${color} rounded-full border border-slate-900 cursor-pointer shadow-[0_0_6px_rgba(0,0,0,0.5)] z-20`;
            point.style.left = `${leftPercent}%`;
            point.style.top = "50%";
            point.style.transform = "translate(-50%, -50%)";

            // Add custom popup info overlay as dataset
            point.title = `[${ev.time}] ${ev.desc}`;

            // Hover details log display
            point.addEventListener("mouseenter", () => {
                point.classList.add("scale-150");
            });
            point.addEventListener("mouseleave", () => {
                point.classList.remove("scale-150");
            });

            point.addEventListener("click", () => {
                addLogMessage(`Zaman Çizelgesi İncelemesi: [${ev.time}] ${ev.desc}`, "DEBUG");
            });

            container.appendChild(point);
        });
    }

    function addTimelineEvent(type, desc) {
        const now = new Date();
        const timeStr = now.toTimeString().split(' ')[0];

        state.timelineEvents.push({
            time: timeStr,
            type: type,
            desc: desc
        });

        // Cap at latest 8 events for visual constraints
        if (state.timelineEvents.length > 8) {
            state.timelineEvents.shift();
        }

        renderTimeline();
    }

    renderTimeline();

    // VLM text auto update simulator (simulating scene change updates)
    const vlmTextEl = document.getElementById("vlm-summary-text");
    const vlmTimeEl = document.getElementById("vlm-time");

    function updateVLMSummary() {
        const newSummary = state.vlmSummaries[state.vlmIndex];
        const now = new Date();
        const timeStr = now.toTimeString().split(' ')[0];

        vlmTimeEl.innerText = timeStr;

        // Typewriter effect simulator
        vlmTextEl.innerHTML = "";
        let i = 0;

        function typeWriter() {
            if (i < newSummary.length) {
                vlmTextEl.innerHTML += newSummary.charAt(i);
                i++;
                setTimeout(typeWriter, 15);
            }
        }
        typeWriter();

        // Increment or cycle summary
        state.vlmIndex = (state.vlmIndex + 1) % state.vlmSummaries.length;
    }

    updateVLMSummary();
    setInterval(updateVLMSummary, 18000); // Scene changes every 18 seconds


    // ----------------------------------------------------
    // 9. AI COPILOT CHAT ENGINE
    // ----------------------------------------------------
    const chatForm = document.getElementById("chat-form");
    const chatInput = document.getElementById("chat-input");
    const chatMessages = document.getElementById("chat-messages");

    function appendMessage(sender, text, isAi = false, isStructured = false) {
        const msgDiv = document.createElement("div");
        msgDiv.className = "flex space-x-2.5";

        const avatarHtml = isAi
            ? `<div class="w-6 h-6 rounded-sm bg-brand-accent/10 border border-brand-accent/30 flex items-center justify-center shrink-0">
                   <i data-lucide="bot" class="w-3.5 h-3.5 text-brand-accent"></i>
               </div>`
            : `<div class="w-6 h-6 rounded-sm bg-brand-cyan/10 border border-brand-cyan/30 flex items-center justify-center shrink-0">
                   <i data-lucide="user" class="w-3.5 h-3.5 text-brand-cyan"></i>
               </div>`;

        const nameColor = isAi ? "text-brand-accent" : "text-brand-cyan";
        const senderName = isAi ? "SYSTEM COPILOT" : "OPERATOR";
        const contentBg = isAi ? "bg-slate-900/60 border border-brand-border/20" : "bg-brand-border/20 border border-brand-border/40";

        msgDiv.innerHTML = `
            ${avatarHtml}
            <div class="flex-1 ${contentBg} rounded p-2.5 text-slate-200">
                <p class="font-bold ${nameColor} mb-1 text-[10px] tracking-wide">${senderName}</p>
                <div class="leading-relaxed whitespace-pre-line">${text}</div>
            </div>
        `;

        chatMessages.appendChild(msgDiv);
        chatMessages.scrollTop = chatMessages.scrollHeight;
        lucide.createIcons();
    }

    function appendTypingIndicator() {
        const indDiv = document.createElement("div");
        indDiv.id = "chat-typing-indicator";
        indDiv.className = "flex space-x-2.5";
        indDiv.innerHTML = `
            <div class="w-6 h-6 rounded-sm bg-brand-accent/10 border border-brand-accent/30 flex items-center justify-center shrink-0">
                <i data-lucide="bot" class="w-3.5 h-3.5 text-brand-accent"></i>
            </div>
            <div class="flex-1 bg-slate-900/60 border border-brand-border/20 rounded p-2.5 text-slate-400 flex items-center space-x-1">
                <span class="w-1.5 h-1.5 bg-brand-accent rounded-full typing-dot"></span>
                <span class="w-1.5 h-1.5 bg-brand-accent rounded-full typing-dot"></span>
                <span class="w-1.5 h-1.5 bg-brand-accent rounded-full typing-dot"></span>
            </div>
        `;
        chatMessages.appendChild(indDiv);
        chatMessages.scrollTop = chatMessages.scrollHeight;
        lucide.createIcons();
        return indDiv;
    }

    function handleChatSubmit(e) {
        if (e) e.preventDefault();
        const text = chatInput.value.trim();
        if (!text) return;

        // Append operator message
        appendMessage("operator", text, false);
        chatInput.value = "";

        // Log query
        addLogMessage(`Operatör AI Copilot Sorgusu: "${text}"`, "INFO");

        // Show typing, then resolve answer
        const typingEl = appendTypingIndicator();

        setTimeout(() => {
            // Remove typing
            typingEl.remove();

            // Generate structured responses
            const lowerText = text.toLowerCase();
            let aiResponse = "";
            let actionDone = false;

            if (lowerText.includes("araç") || lowerText.includes("tespit") || lowerText.includes("kaç")) {
                const vehicleCount = state.detections.filter(d => d.type === "Araç").length;
                const personCount = state.detections.filter(d => d.type === "Kişi").length;
                const lastDetection = state.detections[state.detections.length - 1];

                aiResponse = `<b>Taktiksel Tespit Analizi (Son 5 Dakika):</b>
                - Toplam Araç: <b>${vehicleCount} adet</b> (Sınıfta yüksek kararlılık)
                - Toplam Yaya: <b>${personCount} adet</b>
                
                En son tespit edilen hedef koordinatı (${lastDetection.id}):
                Latitude: <span class="text-brand-accent">${lastDetection.lat.toFixed(5)} N</span>
                Longitude: <span class="text-brand-accent">${lastDetection.lon.toFixed(5)} E</span>
                
                Filtrelenmiş hedef veritabanına erişmek için sol taraftaki tabloyu kullanabilirsiniz.`;
                actionDone = true;
            }
            else if (lowerText.includes("batarya") || lowerText.includes("pil") || lowerText.includes("voltage")) {
                aiResponse = `<b>Enerji Hücresi Durum Raporu:</b>
                - Batarya Seviyesi: <span class="text-brand-accent font-bold">%${state.telemetry.battery.toFixed(0)}</span>
                - Voltaj: <b>${state.telemetry.voltage} V</b>
                - Hücre Sağlığı: <b>EXCELLENT (100%)</b>
                - Kalan Güvenli Devriye Süresi: <b>~16.4 dk</b>
                
                RTL (Home Return) eşiği %15 seviyesine ayarlanmıştır. Herhangi bir hücre dengesizliği rapor edilmemiştir.`;
                actionDone = true;
            }
            else if (lowerText.includes("sahne") || lowerText.includes("neler") || lowerText.includes("vlm")) {
                aiResponse = `<b>VLM Sahne Analiz Verisi:</b>
                "${state.vlmSummaries[(state.vlmIndex - 1 + state.vlmSummaries.length) % state.vlmSummaries.length]}"
                
                Vision Language Model kararları: 
                - Olay Şüphesi: <span class="text-brand-warn font-bold">DÜŞÜK/ORTA</span> (Devriye bölgesinde sınır ihlali)
                - Hava Durumu: Temiz görüş, bulutsuz.`;
                actionDone = true;
            }
            else if (lowerText.includes("rapor") || lowerText.includes("pdf") || lowerText.includes("indir")) {
                aiResponse = `<b>Bölge Taktik Raporu Oluşturuldu:</b>
                Sistem telemetri logları, YOLO koordinat geçmişi ve VLM açıklamaları birleştirilerek rapor dosyası başarıyla derlendi.
                
                Aşağıdaki bağlantıyı kullanarak indirebilirsiniz:
                <button onclick="window.downloadMockReport()" class="mt-2 flex items-center space-x-2 px-3 py-1.5 rounded bg-brand-cyan/20 border border-brand-cyan text-brand-cyan hover:bg-brand-cyan/40 font-bold transition-all text-[10px]">
                    <i data-lucide="download" class="w-3.5 h-3.5"></i>
                    <span>Taktik_Rapor_Ankara.pdf İNDİR</span>
                </button>`;
                actionDone = true;

                // Add timeline event
                addTimelineEvent("command", "AI: Taktik Raporu Derlendi");
            }
            else {
                aiResponse = `Sorgu algılandı. MAVLink telemetrisi ve yer istasyonu kontrol katmanı üzerinden durum analizi yapıldı.
                
                <b>Güncel İHA Durumu:</b>
                - İrtifa: <b>${state.telemetry.alt.toFixed(1)} m</b>
                - Otopilot Modu: <span class="text-brand-warn font-bold">${state.telemetry.mode}</span>
                - Link Gecikmesi: <b>${state.telemetry.latency} ms</b>
                
                Detaylı MAVLink komutu veya koordinat güncellemesi göndermek isterseniz parametre belirtebilirsiniz.`;
            }

            appendMessage("assistant", aiResponse, true);

            // Add action command indicator to timeline if user requested a structured task
            if (actionDone) {
                addTimelineEvent("command", "AI: Taktik Sorgusu Cevaplandı");
            }
        }, 1200);
    }

    chatForm.addEventListener("submit", handleChatSubmit);

    // Suggestion pills click listeners
    const pills = document.querySelectorAll(".suggestion-pill");
    pills.forEach(pill => {
        pill.addEventListener("click", () => {
            chatInput.value = pill.innerText;
            handleChatSubmit();
        });
    });

    // Mock Report Download Function (Bound globally so HTML onclick can access it)
    window.downloadMockReport = () => {
        const textContent = `--- AGENTIC DIGITAL-TWIN GCS TACTICAL REPORT ---
Generated At: ${new Date().toISOString()}
Telemetry Summary:
- Altitude: ${state.telemetry.alt.toFixed(2)} m
- Horizontal Speed: ${state.telemetry.speed.toFixed(2)} m/s
- Battery Level: ${state.telemetry.battery.toFixed(1)}% (${state.telemetry.voltage}V)
- Position: Lat ${state.telemetry.lat.toFixed(6)}, Lon ${state.telemetry.lon.toFixed(6)}
- Autopilot Flight Mode: ${state.telemetry.mode}

Target Detection Logs (YOLOv8):
${state.detections.map(d => `- ID: ${d.id} | Class: ${d.type} | Conf: ${(d.confidence * 100).toFixed(0)}% | GPS: ${d.lat.toFixed(5)}, ${d.lon.toFixed(5)}`).join("\n")}

VLM Scene Context Summary:
"${vlmTextEl.innerText}"
--------------------- END OF REPORT ---------------------`;

        const blob = new Blob([textContent], { type: "text/plain;charset=utf-8" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `Taktik_Rapor_${new Date().toLocaleDateString().replace(/\//g, '-')}.txt`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);

        addLogMessage("Taktiksel durum raporu lokal sürücüye indirildi.", "INFO");
    };

});
