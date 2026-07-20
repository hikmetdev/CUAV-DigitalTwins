"""MCP (Model Context Protocol) stdio istemcisi.

mcp_server.py'yi alt süreç olarak başlatır ve onunla satır-ayrımlı JSON-RPC 2.0
konuşur (MCP stdio taşıması). AI Copilot mimarisindeki rolü:

  app.py (host) ── McpClient ── mcp_server.py (araçlar)
                        │
                 GeminiChatThread (list_tools -> Gemini function declarations,
                                   functionCall -> call_tool)

İstemci uygulama ömrü boyunca TEK sunucu süreci tutar (her sorguda süreç
başlatmamak için); GeminiChatThread'ler onu paylaşır. Erişim kilitle korunur —
zaten aynı anda tek sorgu çalışır (app.py bunu garanti eder), kilit yarış
ihtimaline karşı güvencedir. Yanıt beklemede select ile zaman aşımı uygulanır;
sunucu ölmüşse McpError yükselir ve bir sonraki sorguda yeniden başlatılır.
"""

import os
import sys
import json
import select
import threading
import subprocess

# Proje kökü (threads/'in bir üstü) — mcp_server.py buradadır.
_BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MCP_SERVER_PATH = os.path.join(_BASE_DIR, "mcp_server.py")

PROTOCOL_VERSION = "2025-06-18"
CLIENT_INFO = {"name": "siha-gcs-copilot", "version": "1.0.0"}

# Tek yanıt için bekleme süresi (sn). Araçlar bellek-içi snapshot okuduğundan
# milisaniyeler içinde döner; süre dolarsa sunucu ölmüş/kilitlenmiş demektir.
RESPONSE_TIMEOUT_S = 10.0


class McpError(RuntimeError):
    """MCP taşıma/protokol hatası (sunucu öldü, zaman aşımı, JSON-RPC error)."""


class McpClient:
    def __init__(self, server_path=MCP_SERVER_PATH):
        self.server_path = server_path
        self.proc = None
        self.server_info = {}
        self._next_id = 0
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Yaşam döngüsü
    # ------------------------------------------------------------------
    def start(self):
        """Sunucu sürecini başlatır ve MCP el sıkışmasını yapar."""
        self.proc = subprocess.Popen(
            [sys.executable, self.server_path],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,  # satır tamponlu: her JSON mesajı tek satır
        )
        result = self._request("initialize", {
            "protocolVersion": PROTOCOL_VERSION,
            "capabilities": {},
            "clientInfo": CLIENT_INFO,
        })
        self.server_info = result.get("serverInfo", {})
        self._notify("notifications/initialized", {})
        return self.server_info

    def is_alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if self.proc is None:
            return
        try:
            self.proc.terminate()
            self.proc.wait(timeout=2)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        self.proc = None

    # ------------------------------------------------------------------
    # MCP işlemleri
    # ------------------------------------------------------------------
    def list_tools(self, categories=None):
        """Sunucudaki araç tanımlarını döndürür: [{name, description, inputSchema}].
        categories verilirse (ör. ['telemetry', 'yolo']) yalnızca o kategorideki
        araçlar döner — böylece AI Copilot her sorguda 14 aracın tamamını değil
        göreve uygun alt kümeyi görür (Görev 3). None ise tüm araçlar döner."""
        params = {}
        if categories:
            params["categories"] = list(categories)
        return self._request("tools/list", params).get("tools", [])

    def call_tool(self, name, arguments=None):
        """Aracı çağırır; sonucun text içeriğini JSON olarak çözüp döndürür."""
        result = self._request("tools/call", {
            "name": name,
            "arguments": arguments or {},
        })
        text = "".join(
            c.get("text", "") for c in result.get("content", [])
            if c.get("type") == "text"
        )
        if result.get("isError"):
            return {"error": text or "Araç hatası (ayrıntı yok)."}
        try:
            return json.loads(text)
        except (json.JSONDecodeError, TypeError):
            return {"result": text}

    def update_snapshot(self, snapshot):
        """Güncel GUI durumunu sunucuya iletir (araçlar bunun üzerinde çalışır)."""
        self._notify("notifications/snapshot", {"snapshot": snapshot or {}})

    # ------------------------------------------------------------------
    # JSON-RPC taşıma katmanı
    # ------------------------------------------------------------------
    def _send(self, msg):
        if not self.is_alive():
            raise McpError("MCP sunucusu çalışmıyor.")
        try:
            self.proc.stdin.write(json.dumps(msg, ensure_ascii=False) + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise McpError(f"MCP sunucusuna yazılamadı: {exc}")

    def _notify(self, method, params):
        with self._lock:
            self._send({"jsonrpc": "2.0", "method": method, "params": params})

    def _request(self, method, params):
        with self._lock:
            self._next_id += 1
            req_id = self._next_id
            self._send({"jsonrpc": "2.0", "id": req_id,
                        "method": method, "params": params})
            reply = self._read_response(req_id)
        if "error" in reply:
            err = reply["error"]
            raise McpError(f"MCP hatası ({err.get('code')}): {err.get('message')}")
        return reply.get("result", {})

    def _read_response(self, req_id):
        """Beklenen id'li yanıt gelene dek satır okur (zaman aşımı korumalı).
        Sunucudan istemciye bildirim gelirse (bu sunucu göndermez) atlanır."""
        while True:
            ready, _, _ = select.select([self.proc.stdout], [], [], RESPONSE_TIMEOUT_S)
            if not ready:
                raise McpError("MCP yanıtı zaman aşımına uğradı.")
            line = self.proc.stdout.readline()
            if not line:
                raise McpError("MCP sunucusu bağlantıyı kapattı (süreç öldü).")
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue  # bozuk satır — sıradakini bekle
            if msg.get("id") == req_id:
                return msg
