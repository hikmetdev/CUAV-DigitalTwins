# ----------------------------------------------------------------------
# QSS Stylesheet - Palantir/Anduril Defense UI Theme (Dark Navy/Green)
# ----------------------------------------------------------------------
DARK_THEME_QSS = """
QWidget {
    background-color: #030712;
    color: #e2e8f0;
    font-family: "Inter", "Segoe UI", sans-serif;
}

QFrame.panel {
    background-color: #090d16;
    border: 1px solid #1e293b;
    border-radius: 6px;
}

QFrame.panel-header {
    background-color: #0d1522;
    border-bottom: 1px solid #1e293b;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
}

QLabel.title-label {
    color: #e2e8f0;
    font-weight: bold;
    font-size: 11px;
    font-family: "Roboto Mono", "Courier New";
}

QLabel.badge-green {
    background-color: rgba(16, 185, 129, 0.1);
    color: #10b981;
    border: 1px solid rgba(16, 185, 129, 0.3);
    border-radius: 3px;
    padding: 2px 6px;
    font-size: 9px;
    font-weight: bold;
    font-family: "Roboto Mono";
}

QLabel.badge-cyan {
    background-color: rgba(6, 182, 212, 0.1);
    color: #06b6d4;
    border: 1px solid rgba(6, 182, 212, 0.3);
    border-radius: 3px;
    padding: 2px 6px;
    font-size: 9px;
    font-weight: bold;
    font-family: "Roboto Mono";
}

QLabel.badge-warn {
    background-color: rgba(245, 158, 11, 0.1);
    color: #f59e0b;
    border: 1px solid rgba(245, 158, 11, 0.3);
    border-radius: 3px;
    padding: 2px 6px;
    font-size: 9px;
    font-weight: bold;
    font-family: "Roboto Mono";
}

/* Table Widget styling */
QTableWidget {
    background-color: #090d16;
    gridline-color: rgba(30, 41, 59, 0.3);
    border: none;
    font-family: "Roboto Mono", monospace;
    font-size: 10px;
}

QTableWidget::item {
    padding: 6px;
    border-bottom: 1px solid rgba(30, 41, 59, 0.2);
}

QTableWidget::item:selected {
    background-color: rgba(6, 182, 212, 0.15);
    color: #06b6d4;
}

QHeaderView::section {
    background-color: #0b111c;
    color: #94a3b8;
    border: none;
    border-bottom: 1px solid #1e293b;
    font-weight: bold;
    font-size: 10px;
    padding: 4px;
}

/* Collapsible Console Terminal */
QPlainTextEdit.console {
    background-color: #010409;
    border: 1px solid #1e293b;
    color: #10b981;
    font-family: "Roboto Mono", monospace;
    font-size: 10px;
}

/* Chat text browser */
QTextBrowser.chat-history {
    background-color: #060910;
    border: 1px solid #1e293b;
    border-radius: 4px;
    color: #e2e8f0;
    font-size: 11px;
}

/* Inputs and Forms */
QLineEdit.chat-input {
    background-color: #010409;
    border: 1px solid #1e293b;
    border-radius: 4px;
    padding: 6px 10px;
    color: #f1f5f9;
    font-size: 11px;
    font-family: "Roboto Mono";
}

QLineEdit.chat-input:focus {
    border: 1px solid #10b981;
}

/* Buttons */
QPushButton.btn-primary {
    background-color: #10b981;
    color: #030712;
    border: none;
    border-radius: 4px;
    font-weight: bold;
    padding: 6px 12px;
    font-size: 11px;
}

QPushButton.btn-primary:hover {
    background-color: #059669;
}

QPushButton.btn-secondary {
    background-color: rgba(30, 41, 59, 0.4);
    color: #e2e8f0;
    border: 1px solid #1e293b;
    border-radius: 4px;
    font-size: 11px;
}

QPushButton.btn-secondary:hover {
    background-color: rgba(30, 41, 59, 0.8);
    border: 1px solid #10b981;
}

QPushButton.suggestion-pill {
    background-color: rgba(30, 41, 59, 0.3);
    color: #94a3b8;
    border: 1px solid rgba(30, 41, 59, 0.6);
    border-radius: 10px;
    padding: 3px 8px;
    font-size: 9px;
    font-family: "Roboto Mono";
}

QPushButton.suggestion-pill:hover {
    background-color: rgba(6, 182, 212, 0.1);
    color: #06b6d4;
    border: 1px solid rgba(6, 182, 212, 0.4);
}

/* Scrollbars */
QScrollBar:vertical {
    border: none;
    background: #030712;
    width: 6px;
    margin: 0px;
}

QScrollBar::handle:vertical {
    background: #1e293b;
    min-height: 20px;
    border-radius: 3px;
}

QScrollBar::handle:vertical:hover {
    background: #10b981;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    border: none;
    background: none;
}
"""

# ----------------------------------------------------------------------
# EMBEDDED LEAFLET MAP HTML CODE
# ----------------------------------------------------------------------
LEAFLET_MAP_HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        html, body, #map {
            width: 100%;
            height: 100%;
            margin: 0;
            padding: 0;
            background: #030712;
        }
        /* Custom dark styles for Leaflet elements */
        .leaflet-container {
            background: #030712 !important;
        }
        .leaflet-popup-content-wrapper {
            background: rgba(9, 13, 22, 0.95) !important;
            border: 1px solid rgba(16, 185, 129, 0.4) !important;
            color: #f1f5f9 !important;
            border-radius: 4px !important;
            font-family: monospace;
            font-size: 11px;
            box-shadow: 0 0 15px rgba(0, 0, 0, 0.5) !important;
        }
        .leaflet-popup-tip {
            background: rgba(9, 13, 22, 0.95) !important;
            border: 1px solid rgba(16, 185, 129, 0.4) !important;
        }
        
        /* Tactical glowing markers styling */
        .uav-marker-glow {
            position: relative;
            width: 20px;
            height: 20px;
            background: rgba(6, 182, 212, 0.2);
            border: 2px solid #06b6d4;
            border-radius: 50%;
            box-shadow: 0 0 8px #06b6d4;
        }
        .uav-marker-glow::after {
            content: '';
            position: absolute;
            width: 6px;
            height: 6px;
            background: #06b6d4;
            border-radius: 50%;
            left: 5px;
            top: 5px;
        }
        
        .target-marker-glow {
            position: relative;
            width: 16px;
            height: 16px;
            background: rgba(245, 158, 11, 0.2);
            border: 2px dashed #f59e0b;
            border-radius: 4px;
            box-shadow: 0 0 6px #f59e0b;
        }
        .target-marker-glow::after {
            content: '';
            position: absolute;
            width: 4px;
            height: 4px;
            background: #f59e0b;
            border-radius: 50%;
            left: 4px;
            top: 4px;
        }
        
        .waypoint-marker-glow {
            position: relative;
            width: 10px;
            height: 10px;
            background: rgba(16, 185, 129, 0.1);
            border: 1px solid #10b981;
            transform: rotate(45deg);
            box-shadow: 0 0 4px #10b981;
        }
    </style>
</head>
<body>
    <div id="map"></div>
    <script>
        let map;
        let uavMarker;
        let pathLine;
        let routeLine;
        let targetMarkers = {};

        // Functions exposed globally to Python side
        window.updateUavPosition = function(lat, lon, heading, mode, alt, speed) {
            if (!map || !uavMarker) return;
            const latlng = [lat, lon];
            uavMarker.setLatLng(latlng);
            uavMarker.setPopupContent(`<b>İHA (AD-GCS-01)</b><br>Mod: ${mode}<br>ALT: ${alt.toFixed(1)}m<br>HIZ: ${speed.toFixed(1)} m/s`);
            map.panTo(latlng);
            
            // Only add to trajectory path if the UAV has actually moved
            const latlngs = pathLine.getLatLngs();
            if (latlngs.length === 0) {
                pathLine.addLatLng(latlng);
            } else {
                const lastPoint = latlngs[latlngs.length - 1];
                const dist = map.distance(lastPoint, latlng);
                if (dist > 0.5) { // More than 0.5 meters
                    pathLine.addLatLng(latlng);
                }
            }
        };

        window.clearPath = function() {
            if (pathLine) {
                pathLine.setLatLngs([]);
            }
        };

        window.setWaypoints = function(wpList) {
            if (!map) return;
            const routeCoords = wpList.map(wp => [wp.lat, wp.lon]);
            if (routeLine) {
                routeLine.setLatLngs(routeCoords);
            }
            // Draw waypoint flags
            wpList.forEach((wp, idx) => {
                const wpIcon = L.divIcon({
                    className: 'wp-marker',
                    html: '<div class="waypoint-marker-glow"></div>',
                    iconSize: [10, 10],
                    iconAnchor: [5, 5]
                });
                L.marker([wp.lat, wp.lon], { icon: wpIcon })
                    .addTo(map)
                    .bindPopup(`<b>Waypoint ${idx + 1}</b><br>${wp.name}`);
            });
        };

        window.addTarget = function(id, type, lat, lon, conf) {
            if (!map) return;
            if (targetMarkers[id]) {
                map.removeLayer(targetMarkers[id]);
            }
            
            const targetIcon = L.divIcon({
                className: 'target-marker',
                html: '<div class="target-marker-glow"></div>',
                iconSize: [16, 16],
                iconAnchor: [8, 8]
            });

            targetMarkers[id] = L.marker([lat, lon], { icon: targetIcon })
                .addTo(map)
                .bindPopup(`<b>HEDEF (${id})</b><br>Sınıf: ${type}<br>Güven: %${(conf*100).toFixed(0)}`);
        };

        window.centerOn = function(lat, lon, zoom = 18, id = null) {
            if (map) {
                map.setView([lat, lon], zoom);
                if (id && targetMarkers[id]) {
                    targetMarkers[id].openPopup();
                }
            }
        };

        // Initialize Leaflet when library is loaded
        window.addEventListener('load', function() {
            if (typeof L === 'undefined') {
                console.error("Leaflet library failed to load.");
                return;
            }

            map = L.map('map', {
                zoomControl: false,
                attributionControl: false
            }).setView([39.920782, 32.854115], 16);

            // CartoDB Dark Matter tiles
            L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
                maxZoom: 20
            }).addTo(map);

            // Custom div icons
            const uavIcon = L.divIcon({
                className: 'uav-marker-icon',
                html: '<div class="uav-marker-glow"></div>',
                iconSize: [20, 20],
                iconAnchor: [10, 10]
            });

            // UAV Marker
            uavMarker = L.marker([39.920782, 32.854115], { icon: uavIcon })
                .addTo(map)
                .bindPopup("<b>İHA (AD-GCS-01)</b><br>Mod: AUTO.LOITER<br>ALT: 84.2m");

            // Planned target route line
            routeLine = L.polyline([], {
                color: '#f59e0b',
                weight: 2,
                opacity: 0.75,
                dashArray: '8, 6'
            }).addTo(map);

            // UAV trajectory path line
            pathLine = L.polyline([], {
                color: '#06b6d4',
                weight: 2,
                opacity: 0.6,
                dashArray: '5, 5'
            }).addTo(map);
        });
    </script>
</body>
</html>
"""
