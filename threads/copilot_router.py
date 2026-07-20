"""AI Copilot niyet yönlendirici (router) — YEREL ve HIZLI.

Operatör sorgusunu ağ/model çağrısı OLMADAN sınıflandırır. Amaç iki katlıdır:

  * Görev 2 — basit sohbet (merhaba, naber, teşekkür, "ne yapabilirsin"...)
    MCP'ye HİÇ gitmez: araç yüklenmez, snapshot itilmez, araçsız tek ve hızlı
    bir Gemini çağrısıyla yanıtlanır. Böylece "geç cevap / düşme" sorunu basit
    mesajlarda tümden ortadan kalkar.

  * Görev 3 — veri/komut gerektiren sorgularda 14 aracın tamamı değil, yalnızca
    İLGİLİ kategori(ler) yüklenir (telemetri / yolo / analiz / rapor / uçuş).
    Kısa istem = daha hızlı yanıt + daha az araç karışıklığı.

Yönlendirme tamamen anahtar-kelime (stem) tabanlıdır: Türkçe eklerden dolayı
tam sözcük eşleşmesi yerine sözcük ÖNEKİ eşleşmesi kullanılır ("batarya" ->
"bataryayı", "irtifa" -> "irtifayı", "tespit" -> "tespitler"). Çok kısa ve
genel kökler (hi, ok...) yanlış eşleşmeyi önlemek için TAM eşleşir.

Sınıflandırma güvenli tarafta durur:
  - Hiçbir araç kökü eşleşmez ama sohbet kalıbı eşleşirse  -> sohbet (araçsız).
  - Hiçbir şey eşleşmezse (bilinmeyen ama veri isteği olabilir) -> yalnızca
    OKUMA araçları (telemetry+yolo+analysis); uçuş/rapor gibi yan etkili
    kategoriler asla "tahminle" açılmaz.
"""

import re

# Sunucudaki (mcp_server.TOOL_CATEGORIES) kategori adlarıyla birebir aynı olmalı.
CATEGORIES = ("telemetry", "yolo", "analysis", "report", "flight")

# Hiçbir anahtar kelime eşleşmediğinde (bilinmeyen ama olası veri sorusu)
# açılacak güvenli okuma kümesi. Uçuş (yan etkili) ve rapor (pencere açar)
# asla tahminle eklenmez.
_FALLBACK_CATEGORIES = ["telemetry", "yolo", "analysis"]

# Sözcük belirteçleri (Türkçe harf + rakam dizileri).
_WORD_RE = re.compile(r"[a-zçğıöşü0-9]+")

# Bu kökler prefix değil TAM eşleşir: aşırı kısa/genel oldukları için önek
# eşleşmesi çok yanlış pozitif üretir (ör. "hi" -> "hikaye").
_EXACT_ONLY = {"hi", "ok", "hey", "alo", "bye", "tsk", "tşk", "eyw", "tmm"}


def _normalize(text):
    """Türkçe büyük/küçük harf tuzağını (İ/I) düzelterek küçük harfe indirger."""
    text = (text or "").replace("İ", "i").replace("I", "ı")
    return text.lower()


def _tokens(norm_text):
    return _WORD_RE.findall(norm_text)


def _hit(stems, norm_text, words):
    """stems listesinden herhangi biri metinde/sözcüklerde geçiyor mu?
    - boşluklu kök -> ifade (alt dize) eşleşmesi
    - _EXACT_ONLY kökü -> tam sözcük eşleşmesi
    - diğer          -> sözcük ÖNEKİ eşleşmesi (Türkçe ekleri yakalar)"""
    for stem in stems:
        if " " in stem:
            if stem in norm_text:
                return True
        elif stem in _EXACT_ONLY:
            if stem in words:
                return True
        else:
            if any(w.startswith(stem) for w in words):
                return True
    return False


# ----------------------------------------------------------------------
# ANAHTAR KELİME (KÖK) SÖZLÜKLERİ
# ----------------------------------------------------------------------
_KEYWORDS = {
    "telemetry": [
        "irtifa", "yükseklik", "yukseklik", "hız", "hiz", "sürat", "surat",
        "speed", "batarya", "pil", "şarj", "sarj", "voltaj", "volt", "gerilim",
        "mod", "konum", "koordinat", "gps", "enlem", "boylam", "uydu",
        "yön", "yon", "heading", "telemetri", "mesafe", "uzaklık", "uzaklik",
        "yakıt", "yakit", "nerede", "nereye",
    ],
    "yolo": [
        "tespit", "hedef", "nesne", "cisim", "kutu", "qr", "tabela", "stop",
        "dama", "panel", "nişan", "nisan", "kırmızı", "kirmizi", "mavi",
        "yolo", "yoloe", "sahne", "vlm", "kamera", "görüntü", "goruntu",
        "algıla", "algila", "sınıf", "sinif", "gördü", "gordu", "görülen",
        "gorulen", "obje", "işaret", "isaret",
    ],
    "analysis": [
        "özet", "ozet", "durum", "genel", "analiz", "değerlendir", "degerlendir",
        "taktik", "brifing", "brief",
    ],
    "report": [
        "rapor", "indir", "dosya", "kaydet", "derle", "çıktı", "cikti",
        "döküman", "dokuman", "pdf",
    ],
    "flight": [
        "dön", "don", "döndür", "dondur", "sağa", "saga", "sola",
        "ileri", "ilerle", "geri", "git", "gidip", "yönel", "yonel",
        "yüksel", "yuksel", "alçal", "alcal", "tırman", "tirman",
        "çıkar", "cikar", "hızlan", "hizlan", "yavaşla", "yavasla",
        "başlangıç", "baslangic", "kalkış", "kalkis",
        "rotaya", "otonom", "loiter", "manevra", "uçur", "ucur",
        "yönlendir", "yonlendir",
        "eve dön", "başa dön", "geri dön",  # ifadeler
    ],
}

# Basit sohbet / veri gerektirmeyen kalıplar.
_SMALLTALK = [
    # selamlaşma
    "merhaba", "selam", "günaydın", "gunaydin", "hey", "alo", "hello", "hi",
    "hoşgeldin", "hosgeldin", "iyi günler", "iyi gunler", "iyi akşamlar",
    "iyi aksamlar", "iyi geceler",
    # hal hatır
    "naber", "naber", "nasılsın", "nasilsin", "ne haber", "napıyorsun",
    "napiyorsun", "keyifler", "iyi misin", "iyimisin",
    # teşekkür
    "teşekkür", "tesekkur", "sağ ol", "sag ol", "sağol", "sagol", "eyvallah",
    "tşk", "tsk", "sağolun", "mersi", "thanks", "thank",
    # vedalaşma
    "görüşürüz", "gorusuruz", "hoşça kal", "hosca kal", "bay bay", "bye",
    "kendine iyi bak", "iyi çalışmalar", "iyi calismalar",
    # onay / kısa tepki
    "tamam", "tmm", "peki", "anladım", "anladim", "süper", "super", "harika",
    "mükemmel", "mukemmel", "aynen", "eyw", "ok", "okey", "olur",
    # kimlik / yetenek
    "kimsin", "adın ne", "adin ne", "ne yapabilirsin", "neler yapabilirsin",
    "yardım", "yardim", "ne işe yararsın", "ne ise yararsin", "kendini tanıt",
    "sen nesin",
]


def classify_query(query):
    """Operatör sorgusunu sınıflandırır.

    Döner: {"mode": "chat"|"tools", "categories": [...]}
      - mode == "chat"  -> MCP'ye gidilmez, araçsız hızlı yanıt (Görev 2).
      - mode == "tools" -> yalnızca 'categories' araçları yüklenir (Görev 3).
    """
    norm = _normalize(query)
    words = _tokens(norm)

    matched = [cat for cat in CATEGORIES if _hit(_KEYWORDS[cat], norm, words)]
    if matched:
        return {"mode": "tools", "categories": matched}

    # Araç kökü yok: gerçekten sohbet mi, yoksa bilinmeyen bir veri sorusu mu?
    if _hit(_SMALLTALK, norm, words):
        return {"mode": "chat", "categories": []}

    # Bilinmeyen ama muhtemelen veri isteği: güvenli okuma araçlarını aç.
    return {"mode": "tools", "categories": list(_FALLBACK_CATEGORIES)}
