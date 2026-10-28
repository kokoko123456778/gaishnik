// =====================================================================
//  ИНСПЕКТОР ДПС — мобильная 3D-игра про работу инспектора ГИБДД
//  Один файл, C++14 + SDL2. Всё рисуется и озвучивается кодом.
//  Модели легковых машин извлечены из Mechanic3D.apk (упрощены и
//  упакованы в base64 ниже), грузовик, фура, маршрутка и «классика»
//  строятся процедурно.
// =====================================================================
#include <SDL.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <vector>
#include <string>
#include <algorithm>

static const float PI = 3.14159265358979f;

// ---------------------------------------------------------------------
//  Математика
// ---------------------------------------------------------------------
struct V3 {
    float x, y, z;
    V3() : x(0), y(0), z(0) {}
    V3(float a, float b, float c) : x(a), y(b), z(c) {}
    V3 operator+(const V3& o) const { return V3(x + o.x, y + o.y, z + o.z); }
    V3 operator-(const V3& o) const { return V3(x - o.x, y - o.y, z - o.z); }
    V3 operator*(float s) const { return V3(x * s, y * s, z * s); }
    V3 operator-() const { return V3(-x, -y, -z); }
    V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    V3& operator-=(const V3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    V3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};
static inline float dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(const V3& a, const V3& b) { return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static inline float len(const V3& a) { return sqrtf(dot(a, a)); }
static inline V3 norm(const V3& a) { float l = len(a); return l > 1e-9f ? a * (1.0f / l) : V3(0, 1, 0); }
static inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static inline V3 lerp3(const V3& a, const V3& b, float t) { return a + (b - a) * t; }
static inline float wrapAngle(float a) { while (a > PI) a -= 2 * PI; while (a < -PI) a += 2 * PI; return a; }
static inline float dist2d(const V3& a, const V3& b) { float dx = a.x - b.x, dz = a.z - b.z; return sqrtf(dx * dx + dz * dz); }
static inline float approach(float v, float t, float d) { if (v < t) return v + d > t ? t : v + d; return v - d < t ? t : v - d; }
// направление «вперёд» и «вправо» для угла курса (y вверх, курс 0 = +z, рост курса = поворот направо)
static inline V3 fwdOf(float yaw) { return V3(sinf(yaw), 0, cosf(yaw)); }
static inline V3 rightOf(float yaw) { return V3(cosf(yaw), 0, -sinf(yaw)); }

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed = 12345) : s(seed ? seed : 1) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    float f() { return (next() & 0xFFFFFF) / 16777216.0f; }
    int i(int n) { return n <= 0 ? 0 : (int)(next() % (uint32_t)n); }
    float range(float a, float b) { return a + (b - a) * f(); }
    bool chance(float p) { return f() < p; }
};
static Rng grng(1234567);

static inline uint32_t hash32(uint32_t x) { x ^= x >> 16; x *= 0x7feb352d; x ^= x >> 15; x *= 0x846ca68b; x ^= x >> 16; return x; }

// ---------------------------------------------------------------------
//  UTF-8
// ---------------------------------------------------------------------
static std::vector<uint32_t> utf8Decode(const char* s) {
    std::vector<uint32_t> out;
    const unsigned char* p = (const unsigned char*)s;
    while (*p) {
        uint32_t c = *p;
        if (c < 0x80) { out.push_back(c); p++; }
        else if ((c & 0xE0) == 0xC0 && p[1]) { out.push_back(((c & 0x1F) << 6) | (p[1] & 0x3F)); p += 2; }
        else if ((c & 0xF0) == 0xE0 && p[1] && p[2]) { out.push_back(((c & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F)); p += 3; }
        else if ((c & 0xF8) == 0xF0 && p[1] && p[2] && p[3]) { out.push_back(((c & 0x07) << 18) | ((p[1] & 0x3F) << 12) | ((p[2] & 0x3F) << 6) | (p[3] & 0x3F)); p += 4; }
        else p++;
    }
    return out;
}
static int utf8Len(const char* s) { int n = 0; while (*s) { if ((*s & 0xC0) != 0x80) n++; s++; } return n; }

static std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap; va_start(ap, f); vsnprintf(buf, sizeof(buf), f, ap); va_end(ap);
    return std::string(buf);
}

// ---------------------------------------------------------------------
//  Пиксельный шрифт 5x7 с кириллицей (только заглавные — строчные приводятся)
// ---------------------------------------------------------------------
struct GlyphDef { uint32_t cp; const char* r; };
static const GlyphDef GLYPHS[] = {
    {' ', "..................................."},
    {'0', ".###.#...##..###.#.###..##...#.###."},
    {'1', "..#...##....#....#....#....#...###."},
    {'2', ".###.#...#....#...#...#...#...#####"},
    {'3', "#####...#...#.....#.....##...#.###."},
    {'4', "...#...##..#.#.#..#.#####...#....#."},
    {'5', "######....####.....#....##...#.###."},
    {'6', "..##..#...#....####.#...##...#.###."},
    {'7', "#####....#...#...#...#....#....#..."},
    {'8', ".###.#...##...#.###.#...##...#.###."},
    {'9', ".###.#...##...#.####....#...#..##.."},
    {'.', ".........................##...##..."},
    {',', "....................##....#...#...."},
    {':', ".....##...##.........##...##......."},
    {';', ".....##...##.........##....#...#..."},
    {'!', "..#....#....#....#....#.........#.."},
    {'?', ".###.#...#....#...#...#.........#.."},
    {'-', "...............#####..............."},
    {'+', ".......#....#..#####..#....#......."},
    {'=', "..........#####.....#####.........."},
    {'/', "....#....#...#...#...#...#....#...."},
    {'(', "...#...#...#....#....#.....#.....#."},
    {')', ".#.....#.....#....#....#...#...#..."},
    {'%', "##..###..#...#...#...#...#..###..##"},
    {'"', ".#.#..#.#.........................."},
    {'\'', "..#....#..........................."},
    {'<', "...#...#...#...#.....#.....#.....#."},
    {'>', ".#.....#.....#.....#...#...#...#..."},
    {'*', ".....#.#.#.###.#####.###.#.#.#....."},
    {'_', "..............................#####"},
    {'#', ".#.#.#####.#.#..#.#..#.#.#####.#.#."},
    {'[', ".###..#....#....#....#....#....###."},
    {']', ".###....#....#....#....#....#..###."},
    {'|', "..#....#....#....#....#....#....#.."},
    {'@', ".###.#...##.####.#.##.####.....###."},
    {'R', "####.#...##...#####.#.#..#..#.#...#"},
    {'U', "#...##...##...##...##...##...#.###."},
    {'S', ".#####....#.....###.....#....#####."},
    {'~', "..........#...#.#.#...#............"},
    {0x2116, "#..#.##.#.#.##.#..#.#..#......#####"}, // №
    {0x20BD, "####.#...##...#####.#....###..#...."}, // ₽
    {0x00B0, ".##..#..#..##......................"}, // °
    {0x00AB, "..........#.#.#.#.#.#...#.#...#.#.."}, // «
    {0x00BB, ".........#.#...#.#...#.#.#.#.#.#..."}, // »
    {0x2192, ".......#.....#.#####...#...#......."}, // →
    {0x2190, ".......#...#...#####.#.....#......."}, // ←
    {0x2191, "..#...###.#.#.#..#....#....#....#.."}, // ↑
    {0x2193, "..#....#....#....#..#.#.#.###...#.."}, // ↓
    {0x410, ".###.#...##...#######...##...##...#"}, // А
    {0x411, "######....#....####.#...##...#####."}, // Б
    {0x412, "####.#...##...#####.#...##...#####."}, // В
    {0x413, "######....#....#....#....#....#...."}, // Г
    {0x414, "..##..#.#..#.#.#..#.#..#.######...#"}, // Д
    {0x415, "######....#....####.#....#....#####"}, // Е
    {0x401, ".#.#.######....####.#....#....#####"}, // Ё
    {0x416, "#.#.##.#.#.###...#...###.#.#.##.#.#"}, // Ж
    {0x417, ".###.#...#....#..##.....##...#.###."}, // З
    {0x418, "#...##...##..###.#.###..##...##...#"}, // И
    {0x419, ".#.#.#...##..###.#.###..##...##...#"}, // Й
    {0x41A, "#...##..#.#.#..##...#.#..#..#.#...#"}, // К
    {0x41B, "..###.#..#.#..#.#..#.#..##...##...#"}, // Л
    {0x41C, "#...###.###.#.##.#.##...##...##...#"}, // М
    {0x41D, "#...##...##...#######...##...##...#"}, // Н
    {0x41E, ".###.#...##...##...##...##...#.###."}, // О
    {0x41F, "######...##...##...##...##...##...#"}, // П
    {0x420, "####.#...##...#####.#....#....#...."}, // Р
    {0x421, ".###.#...##....#....#....#...#.###."}, // С
    {0x422, "#####..#....#....#....#....#....#.."}, // Т
    {0x423, "#...##...##...#.####....##...#.###."}, // У
    {0x424, "..#...###.#.#.##.#.##.#.#.###...#.."}, // Ф
    {0x425, "#...##...#.#.#...#...#.#.#...##...#"}, // Х
    {0x426, "#..#.#..#.#..#.#..#.#..#.#####....#"}, // Ц
    {0x427, "#...##...##...#.####....#....#....#"}, // Ч
    {0x428, "#.#.##.#.##.#.##.#.##.#.##.#.######"}, // Ш
    {0x429, "#.#.##.#.##.#.##.#.##.#.######....#"}, // Щ
    {0x42A, "##....#....#....###..#..#.#..#.###."}, // Ъ
    {0x42B, "#...##...##...###..##.#.##.#.###..#"}, // Ы
    {0x42C, "#....#....#....####.#...##...#####."}, // Ь
    {0x42D, ".###.#...#....#.####....##...#.###."}, // Э
    {0x42E, "#..#.#.#.##.#.####.##.#.##.#.##..#."}, // Ю
    {0x42F, ".#####...##...#.####..#.#.#..##...#"}, // Я
};
static const int NGLYPHS = sizeof(GLYPHS) / sizeof(GLYPHS[0]);

static int glyphIndex(uint32_t c) {
    // строчные кириллица -> заглавные
    if (c >= 0x430 && c <= 0x44F) c -= 0x20;
    if (c == 0x451) c = 0x401;
    // латиница -> похожие кириллические или цифры
    if (c == 0x2014 || c == 0x2013) c = '-';
    if (c == 0x2022) c = '*';
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c == 'R' || c == 'U' || c == 'S') { /* есть свои глифы */ }
    else if (c >= 'A' && c <= 'Z') {
        static const uint32_t lat[26] = {0x410, 0x412, 0x421, 0x414, 0x415, 0x424, 0x413, 0x41D, '1', 0x419, 0x41A, 0x41B, 0x41C, 0x418, 0x41E, 0x420, 0x41E, 0x42F, 0x421, 0x422, 0x426, 0x423, 0x428, 0x425, 0x423, 0x417};
        c = lat[c - 'A'];
    }
    static int cache[0x500]; static bool inited = false;
    if (!inited) {
        for (int i = 0; i < 0x500; i++) cache[i] = -1;
        for (int i = 0; i < NGLYPHS; i++) if (GLYPHS[i].cp < 0x500) cache[GLYPHS[i].cp] = i;
        inited = true;
    }
    if (c < 0x500) { int g = cache[c]; return g >= 0 ? g : 16; }
    for (int i = 0; i < NGLYPHS; i++) if (GLYPHS[i].cp == c) return i;
    return 16; // '?'
}
static bool glyphPixel(int g, int x, int y) {
    const char* r = GLYPHS[g].r;
    int n = (int)strlen(r);
    int idx = y * 5 + x;
    return idx < n && r[idx] == '#';
}

// ---------------------------------------------------------------------
//  Текстурный атлас (512x512): белый пиксель, шрифт, спрайты, номера
// ---------------------------------------------------------------------
static const int ATL = 512;
static uint32_t* atlasPix = nullptr;
static SDL_Texture* atlasTex = nullptr;     // обычное смешивание
static SDL_Texture* atlasTexAdd = nullptr;  // аддитивное (свечение)
static SDL_Renderer* gRen = nullptr;
static SDL_Window* gWin = nullptr;
static int SW = 1280, SH = 720;

static inline uint32_t PIX(int r, int g, int b, int a = 255) { return ((uint32_t)a << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b; }
static void atlasSet(int x, int y, uint32_t c) { if (x >= 0 && y >= 0 && x < ATL && y < ATL) atlasPix[y * ATL + x] = c; }

// регионы атласа
static const int GLYPH_X0 = 8, GLYPH_Y0 = 0, GLYPH_PER_ROW = 80;
static const int SOFT_X = 0, SOFT_Y = 32, SOFT_S = 64;      // мягкий круг
static const int DISC_X = 64, DISC_Y = 32, DISC_S = 64;     // ровный круг
static const int WIN_X = 128, WIN_Y = 32;                    // окна 16x16 x4
static const int CHECK_X = 192, CHECK_Y = 32;                // шашки такси 16x8
static const int DPS_X = 208, DPS_Y = 32, DPS_W = 40, DPS_H = 12; // надпись ДПС
static const int POL_X = 256, POL_Y = 32;                    // ПОЛИЦИЯ
static const int RAIN_X = 304, RAIN_Y = 32;                  // капля
static const int PLATE_X0 = 0, PLATE_Y0 = 96, PLATE_W = 64, PLATE_H = 16, PLATES_PER_ROW = 8, PLATE_SLOTS = 160;

struct UV { float u, v; };
static inline SDL_FPoint uvp(float px, float py) { SDL_FPoint p; p.x = px / ATL; p.y = py / ATL; return p; }
static SDL_FPoint UV_WHITE;

static void glyphCell(int g, int& gx, int& gy) { gx = GLYPH_X0 + (g % GLYPH_PER_ROW) * 6; gy = GLYPH_Y0 + (g / GLYPH_PER_ROW) * 8; }

static void atlasDrawText(int x, int y, const char* s, uint32_t col) {
    std::vector<uint32_t> cps = utf8Decode(s);
    for (size_t i = 0; i < cps.size(); i++) {
        int g = glyphIndex(cps[i]);
        for (int yy = 0; yy < 7; yy++) for (int xx = 0; xx < 5; xx++) if (glyphPixel(g, xx, yy)) atlasSet(x + xx, y + yy, col);
        x += 6;
    }
}

static void buildAtlas() {
    atlasPix = (uint32_t*)calloc(ATL * ATL, 4);
    for (int y = 0; y < 4; y++) for (int x = 0; x < 4; x++) atlasSet(x, y, PIX(255, 255, 255));
    UV_WHITE = uvp(2, 2);
    for (int g = 0; g < NGLYPHS; g++) {
        int gx, gy; glyphCell(g, gx, gy);
        for (int yy = 0; yy < 7; yy++) for (int xx = 0; xx < 5; xx++)
            if (glyphPixel(g, xx, yy)) atlasSet(gx + xx, gy + yy, PIX(255, 255, 255));
    }
    for (int y = 0; y < SOFT_S; y++) for (int x = 0; x < SOFT_S; x++) {
        float dx = (x + 0.5f) / SOFT_S * 2 - 1, dy = (y + 0.5f) / SOFT_S * 2 - 1;
        float d = sqrtf(dx * dx + dy * dy);
        float a = clampf(1 - d, 0, 1); a = a * a * (3 - 2 * a);
        atlasSet(SOFT_X + x, SOFT_Y + y, PIX(255, 255, 255, (int)(a * 255)));
        float a2 = clampf((1 - d) * DISC_S * 0.5f, 0, 1);
        atlasSet(DISC_X + x, DISC_Y + y, PIX(255, 255, 255, (int)(a2 * 255)));
    }
    // окна: 0 — днём (стекло), 1 — горит ночью, 2 — тёмное ночью, 3 — витрина
    for (int t = 0; t < 4; t++) for (int y = 0; y < 16; y++) for (int x = 0; x < 16; x++) {
        bool win = (x >= 3 && x <= 12 && y >= 3 && y <= 12);
        bool frame = win && (x == 7 || x == 8 || y == 7);
        uint32_t c = PIX(255, 255, 255);
        if (t == 3) { win = (x >= 1 && x <= 14 && y >= 2 && y <= 15); frame = win && (x == 8); }
        if (win) {
            if (frame) c = PIX(200, 200, 200);
            else if (t == 0) { int g = 120 + (y < 7 ? 40 : 0) - x * 3; c = PIX(g - 40, g, g + 40); }
            else if (t == 1) c = PIX(255, 220, 140);
            else if (t == 2) c = PIX(40, 45, 60);
            else c = PIX(255, 235, 180);
        }
        atlasSet(WIN_X + t * 16 + x, WIN_Y + y, c);
    }
    for (int y = 0; y < 8; y++) for (int x = 0; x < 16; x++) {
        bool b = (((x / 2) + (y / 2)) & 1) != 0;
        atlasSet(CHECK_X + x, CHECK_Y + y, b ? PIX(20, 20, 20) : PIX(255, 220, 40));
    }
    // ДПС: синие буквы x2 на прозрачном фоне
    {
        std::vector<uint32_t> cps = utf8Decode("ДПС");
        int x0 = 2;
        for (size_t i = 0; i < cps.size(); i++) {
            int g = glyphIndex(cps[i]);
            for (int yy = 0; yy < 7; yy++) for (int xx = 0; xx < 5; xx++) if (glyphPixel(g, xx, yy))
                for (int k = 0; k < 4; k++) atlasSet(DPS_X + x0 + xx * 2 + (k & 1), DPS_Y + yy + (k >> 1) * 0 + 2, PIX(20, 50, 150));
            x0 += 12;
        }
        atlasDrawText(POL_X + 3, POL_Y + 1, "ПОЛИЦИЯ", PIX(20, 50, 150));
    }
    for (int y = 0; y < 16; y++) for (int x = 0; x < 4; x++) {
        float a = (1 - fabsf(x - 1.5f) / 2.0f) * (y / 16.0f);
        atlasSet(RAIN_X + x, RAIN_Y + y, PIX(200, 210, 230, (int)(a * 200)));
    }
}
static void atlasUpload(int x, int y, int w, int h) {
    SDL_Rect r = {x, y, w, h};
    SDL_UpdateTexture(atlasTex, &r, atlasPix + y * ATL + x, ATL * 4);
    SDL_UpdateTexture(atlasTexAdd, &r, atlasPix + y * ATL + x, ATL * 4);
}
static void createAtlasTextures() {
    atlasTex = SDL_CreateTexture(gRen, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, ATL, ATL);
    atlasTexAdd = SDL_CreateTexture(gRen, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, ATL, ATL);
    SDL_SetTextureBlendMode(atlasTex, SDL_BLENDMODE_BLEND);
    SDL_SetTextureBlendMode(atlasTexAdd, SDL_BLENDMODE_ADD);
    SDL_SetTextureScaleMode(atlasTex, SDL_ScaleModeNearest);
    SDL_SetTextureScaleMode(atlasTexAdd, SDL_ScaleModeNearest);
    SDL_UpdateTexture(atlasTex, nullptr, atlasPix, ATL * 4);
    SDL_UpdateTexture(atlasTexAdd, nullptr, atlasPix, ATL * 4);
}

// номера: буквы, разрешённые на российских номерах
static const char* PLATE_LETTERS[12] = {"А", "В", "Е", "К", "М", "Н", "О", "Р", "С", "Т", "У", "Х"};
static std::string randomPlate(Rng& r, int& region) {
    static const int regions[] = {66, 96, 196, 74, 174, 72, 45, 59, 159, 77, 50, 102, 116, 63, 55};
    region = regions[r.i(15)];
    return std::string(PLATE_LETTERS[r.i(12)]) + fmt("%d%d%d", r.i(10), r.i(10), 1 + r.i(9)) + PLATE_LETTERS[r.i(12)] + PLATE_LETTERS[r.i(12)];
}
static void platePixels(int slot, const std::string& main, int region, bool police) {
    int px = PLATE_X0 + (slot % PLATES_PER_ROW) * PLATE_W, py = PLATE_Y0 + (slot / PLATES_PER_ROW) * PLATE_H;
    uint32_t bg = police ? PIX(30, 70, 170) : PIX(245, 245, 245);
    uint32_t fg = police ? PIX(255, 255, 255) : PIX(15, 15, 15);
    for (int y = 0; y < PLATE_H; y++) for (int x = 0; x < PLATE_W; x++) {
        bool border = (x == 0 || y == 0 || x == PLATE_W - 1 || y == PLATE_H - 1);
        bool sep = (x == 45);
        atlasSet(px + x, py + y, border ? PIX(10, 10, 10) : (sep ? fg : bg));
    }
    atlasDrawText(px + 4, py + 5, main.c_str(), fg);
    std::string rs = fmt("%d", region);
    atlasDrawText(px + 47 + (rs.size() == 2 ? 3 : 0), py + 3, rs.c_str(), fg);
    // флажок
    for (int x = 0; x < 9; x++) { atlasSet(px + 50 + x, py + 11, PIX(255, 255, 255)); atlasSet(px + 50 + x, py + 12, PIX(30, 60, 200)); atlasSet(px + 50 + x, py + 13, PIX(210, 30, 30)); }
    atlasUpload(px, py, PLATE_W, PLATE_H);
}
static SDL_FPoint plateUV(int slot, float fx, float fy) {
    int px = PLATE_X0 + (slot % PLATES_PER_ROW) * PLATE_W, py = PLATE_Y0 + (slot / PLATES_PER_ROW) * PLATE_H;
    return uvp(px + 0.5f + fx * (PLATE_W - 1), py + 0.5f + fy * (PLATE_H - 1));
}

// ---------------------------------------------------------------------
//  Цвета
// ---------------------------------------------------------------------
struct Col { float r, g, b; Col() : r(0), g(0), b(0) {} Col(float a, float b2, float c) : r(a), g(b2), b(c) {} };
static inline Col operator*(const Col& a, float s) { return Col(a.r * s, a.g * s, a.b * s); }
static inline Col operator+(const Col& a, const Col& b) { return Col(a.r + b.r, a.g + b.g, a.b + b.b); }
static inline Col mulc(const Col& a, const Col& b) { return Col(a.r * b.r, a.g * b.g, a.b * b.b); }
static inline Col lerpc(const Col& a, const Col& b, float t) { return Col(lerpf(a.r, b.r, t), lerpf(a.g, b.g, t), lerpf(a.b, b.b, t)); }
static inline SDL_Color toSDL(const Col& c, float a = 1) {
    SDL_Color o;
    o.r = (Uint8)(clampf(c.r, 0, 1) * 255); o.g = (Uint8)(clampf(c.g, 0, 1) * 255); o.b = (Uint8)(clampf(c.b, 0, 1) * 255); o.a = (Uint8)(clampf(a, 0, 1) * 255);
    return o;
}
static inline SDL_Color RGBA(int r, int g, int b, int a = 255) { SDL_Color c; c.r = (Uint8)r; c.g = (Uint8)g; c.b = (Uint8)b; c.a = (Uint8)a; return c; }
static inline Col hexc(uint32_t h) { return Col(((h >> 16) & 255) / 255.f, ((h >> 8) & 255) / 255.f, (h & 255) / 255.f); }

// ---------------------------------------------------------------------
//  Освещение и туман
// ---------------------------------------------------------------------
struct Lighting {
    V3 sun = norm(V3(0.4f, 0.8f, 0.3f));
    Col sunCol = Col(1, 0.97f, 0.9f);
    Col amb = Col(0.45f, 0.48f, 0.55f);
    Col fog = Col(0.7f, 0.78f, 0.86f);
    Col skyTop = Col(0.35f, 0.55f, 0.85f);
    Col skyHor = Col(0.75f, 0.82f, 0.9f);
    float fogStart = 60, fogEnd = 320;
    float night = 0;     // 0 день .. 1 ночь
    float rain = 0;
    // фары игрока для подсветки ночью
    V3 hlPos, hlDir; float hlOn = 0;
};
static Lighting L;

static inline float fogFactor(float d) { return clampf((d - L.fogStart) / (L.fogEnd - L.fogStart), 0, 1); }

static inline Col shadeN(const Col& base, const V3& n, const V3& wp, float emissive = 0) {
    float d = dot(n, L.sun); if (d < 0) d = 0;
    Col c = mulc(base, L.amb + L.sunCol * (d * (1 - L.night * 0.85f)));
    if (L.hlOn > 0) {
        V3 dv = wp - L.hlPos; float dd = len(dv);
        if (dd < 45 && dd > 0.1f) {
            float cs = dot(dv * (1 / dd), L.hlDir);
            if (cs > 0.82f) {
                float k = (cs - 0.82f) / 0.18f * (1 - dd / 45) * L.hlOn * 1.3f;
                float nd = -dot(n, dv * (1 / dd)); if (nd < 0.2f) nd = 0.2f;
                c = c + mulc(base, Col(1, 0.95f, 0.8f)) * (k * nd);
            }
        }
    }
    if (emissive > 0) c = lerpc(c, base, emissive);
    return c;
}

// ---------------------------------------------------------------------
//  3D-рендерер: сортировка треугольников по глубине (алгоритм художника)
//  поверх SDL_RenderGeometry одним атласом. Слои: «плоский» (земля, дороги,
//  разметка, тени — в порядке подачи) и «сортируемый» (объекты).
// ---------------------------------------------------------------------
enum { TEX_BLEND = 0, TEX_ADD = 1 };
struct RTri { SDL_Vertex v[3]; float depth; uint8_t tex; };

struct Renderer {
    std::vector<SDL_Vertex> flat, flatAdd;
    std::vector<RTri> tris;
    std::vector<uint64_t> keys;
    std::vector<SDL_Vertex> batch;
    V3 pos, R, U, F;
    float yaw = 0, pitch = 0, focal = 600, cx = 640, cy = 360, nearZ = 0.2f, farZ = 400;
    int stat = 0;

    void setCamera(const V3& p, float yw, float pt, float fovDeg) {
        pos = p; yaw = yw; pitch = pt;
        V3 f0 = fwdOf(yw);
        F = V3(f0.x * cosf(pt), sinf(pt), f0.z * cosf(pt));
        R = rightOf(yw);
        U = cross(F, R); // при R=+x и F=+z получаем U=+y
        cx = SW * 0.5f; cy = SH * 0.5f;
        focal = (SW * 0.5f) / tanf(fovDeg * PI / 360.0f);
    }
    inline V3 toView(const V3& p) const { V3 d = p - pos; return V3(dot(d, R), dot(d, U), dot(d, F)); }
    inline bool project(const V3& w, float& sx, float& sy, float& z) const {
        V3 v = toView(w); z = v.z;
        if (v.z < nearZ) return false;
        sx = cx + v.x * focal / v.z; sy = cy - v.y * focal / v.z; return true;
    }
    // быстрая проверка сферы на видимость
    inline bool sphereVisible(const V3& c, float r) const {
        V3 v = toView(c);
        if (v.z < -r || v.z - r > farZ) return false;
        float zz = v.z > 0.1f ? v.z : 0.1f;
        float sx = v.x * focal / zz, sy = v.y * focal / zz, rr = r * focal / zz;
        if (sx - rr > cx + 2 || sx + rr < -cx - 2) return v.z < r;
        if (sy - rr > cy + 2 || sy + rr < -cy - 2) return v.z < r;
        return true;
    }
    void begin() { flat.clear(); flatAdd.clear(); tris.clear(); }

    // треугольник в координатах вида; mode: 0 плоский, 1 плоский-аддитивный, 2 сорт., 3 сорт.-аддитивный
    void triView(V3 a, V3 b, V3 c, SDL_Color ca, SDL_Color cb, SDL_Color cc, SDL_FPoint ta, SDL_FPoint tb, SDL_FPoint tc, int mode, int cull, float bias) {
        V3 p[3] = {a, b, c}; SDL_Color col[3] = {ca, cb, cc}; SDL_FPoint uv[3] = {ta, tb, tc};
        int behind = (a.z < nearZ) + (b.z < nearZ) + (c.z < nearZ);
        if (behind == 3) return;
        if (behind == 0) { emit(p, col, uv, mode, cull, bias); return; }
        // отсечение ближней плоскостью
        V3 op[4]; SDL_Color oc[4]; SDL_FPoint ou[4]; int n = 0;
        for (int i = 0; i < 3; i++) {
            int j = (i + 1) % 3;
            bool ii = p[i].z >= nearZ, jj = p[j].z >= nearZ;
            if (ii) { op[n] = p[i]; oc[n] = col[i]; ou[n] = uv[i]; n++; }
            if (ii != jj) {
                float t = (nearZ - p[i].z) / (p[j].z - p[i].z);
                op[n] = lerp3(p[i], p[j], t);
                oc[n].r = (Uint8)lerpf(col[i].r, col[j].r, t); oc[n].g = (Uint8)lerpf(col[i].g, col[j].g, t);
                oc[n].b = (Uint8)lerpf(col[i].b, col[j].b, t); oc[n].a = (Uint8)lerpf(col[i].a, col[j].a, t);
                ou[n].x = lerpf(uv[i].x, uv[j].x, t); ou[n].y = lerpf(uv[i].y, uv[j].y, t);
                n++;
            }
        }
        if (n >= 3) {
            V3 q[3] = {op[0], op[1], op[2]}; SDL_Color qc[3] = {oc[0], oc[1], oc[2]}; SDL_FPoint qu[3] = {ou[0], ou[1], ou[2]};
            emit(q, qc, qu, mode, cull, bias);
        }
        if (n == 4) {
            V3 q[3] = {op[0], op[2], op[3]}; SDL_Color qc[3] = {oc[0], oc[2], oc[3]}; SDL_FPoint qu[3] = {ou[0], ou[2], ou[3]};
            emit(q, qc, qu, mode, cull, bias);
        }
    }
    inline void emit(const V3* p, const SDL_Color* c, const SDL_FPoint* uv, int mode, int cull, float bias) {
        float sx[3], sy[3];
        for (int i = 0; i < 3; i++) { float iz = focal / p[i].z; sx[i] = cx + p[i].x * iz; sy[i] = cy - p[i].y * iz; }
        if (sx[0] < 0 && sx[1] < 0 && sx[2] < 0) return;
        if (sx[0] > SW && sx[1] > SW && sx[2] > SW) return;
        if (sy[0] < 0 && sy[1] < 0 && sy[2] < 0) return;
        if (sy[0] > SH && sy[1] > SH && sy[2] > SH) return;
        if (cull) {
            float area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
            if (area * cull > 0) return;
        }
        SDL_Vertex v[3];
        for (int i = 0; i < 3; i++) { v[i].position.x = sx[i]; v[i].position.y = sy[i]; v[i].color = c[i]; v[i].tex_coord = uv[i]; }
        if (mode == 0) { flat.push_back(v[0]); flat.push_back(v[1]); flat.push_back(v[2]); return; }
        if (mode == 1) { flatAdd.push_back(v[0]); flatAdd.push_back(v[1]); flatAdd.push_back(v[2]); return; }
        RTri t; t.v[0] = v[0]; t.v[1] = v[1]; t.v[2] = v[2];
        t.depth = (p[0].z + p[1].z + p[2].z) * (1.0f / 3.0f) + bias;
        t.tex = (mode == 3) ? TEX_ADD : TEX_BLEND;
        tris.push_back(t);
    }
    // удобные обёртки в мировых координатах
    void triW(const V3& a, const V3& b, const V3& c, SDL_Color col, int mode = 2, int cull = 0, float bias = 0) {
        triView(toView(a), toView(b), toView(c), col, col, col, UV_WHITE, UV_WHITE, UV_WHITE, mode, cull, bias);
    }
    void quadW(const V3& a, const V3& b, const V3& c, const V3& d, SDL_Color col, int mode = 2, int cull = 0, float bias = 0) {
        V3 va = toView(a), vb = toView(b), vc = toView(c), vd = toView(d);
        triView(va, vb, vc, col, col, col, UV_WHITE, UV_WHITE, UV_WHITE, mode, cull, bias);
        triView(va, vc, vd, col, col, col, UV_WHITE, UV_WHITE, UV_WHITE, mode, cull, bias);
    }
    void quadWC(const V3& a, const V3& b, const V3& c, const V3& d, SDL_Color ca, SDL_Color cb, SDL_Color cc, SDL_Color cd, int mode) {
        V3 va = toView(a), vb = toView(b), vc = toView(c), vd = toView(d);
        triView(va, vb, vc, ca, cb, cc, UV_WHITE, UV_WHITE, UV_WHITE, mode, 0, 0);
        triView(va, vc, vd, ca, cc, cd, UV_WHITE, UV_WHITE, UV_WHITE, mode, 0, 0);
    }
    void quadWT(const V3& a, const V3& b, const V3& c, const V3& d, SDL_Color col, SDL_FPoint ta, SDL_FPoint tb, SDL_FPoint tc, SDL_FPoint td, int mode = 2, int cull = 0, float bias = 0) {
        V3 va = toView(a), vb = toView(b), vc = toView(c), vd = toView(d);
        triView(va, vb, vc, col, col, col, ta, tb, tc, mode, cull, bias);
        triView(va, vc, vd, col, col, col, ta, tc, td, mode, cull, bias);
    }
    // спрайт-billboard (обращён к камере)
    void billboard(const V3& c, float w, float h, SDL_Color col, int region, int mode, float bias = 0) {
        V3 v = toView(c);
        if (v.z < nearZ + 0.05f) return;
        float px = 0, py = 0, ps = 64;
        if (region == 0) { px = SOFT_X; py = SOFT_Y; ps = SOFT_S; } else { px = DISC_X; py = DISC_Y; ps = DISC_S; }
        SDL_FPoint t0 = uvp(px + 0.5f, py + 0.5f), t1 = uvp(px + ps - 0.5f, py + 0.5f), t2 = uvp(px + ps - 0.5f, py + ps - 0.5f), t3 = uvp(px + 0.5f, py + ps - 0.5f);
        V3 a(v.x - w, v.y + h, v.z), b(v.x + w, v.y + h, v.z), cc(v.x + w, v.y - h, v.z), d(v.x - w, v.y - h, v.z);
        triView(a, b, cc, col, col, col, t0, t1, t2, mode, 0, bias);
        triView(a, cc, d, col, col, col, t0, t2, t3, mode, 0, bias);
    }
    // спрайт, лежащий на земле (тень, свет фар)
    void groundSprite(const V3& c, const V3& ax, const V3& az, SDL_Color col, int mode, int region = 0) {
        float px = region == 0 ? SOFT_X : DISC_X, py = region == 0 ? SOFT_Y : DISC_Y, ps = 64;
        SDL_FPoint t0 = uvp(px + 0.5f, py + 0.5f), t1 = uvp(px + ps - 0.5f, py + 0.5f), t2 = uvp(px + ps - 0.5f, py + ps - 0.5f), t3 = uvp(px + 0.5f, py + ps - 0.5f);
        quadWT(c - ax + az, c + ax + az, c + ax - az, c - ax - az, col, t0, t1, t2, t3, mode);
    }

    void flushBatch(SDL_Texture* t) {
        if (batch.empty()) return;
        SDL_RenderGeometry(gRen, t, batch.data(), (int)batch.size(), nullptr, 0);
        batch.clear();
    }
    void draw() {
        if (!flat.empty()) SDL_RenderGeometry(gRen, atlasTex, flat.data(), (int)flat.size(), nullptr, 0);
        if (!flatAdd.empty()) SDL_RenderGeometry(gRen, atlasTexAdd, flatAdd.data(), (int)flatAdd.size(), nullptr, 0);
        size_t n = tris.size();
        keys.resize(n);
        for (size_t i = 0; i < n; i++) {
            float d = tris[i].depth; if (d < 0) d = 0;
            uint32_t bits; memcpy(&bits, &d, 4);
            keys[i] = ((uint64_t)(0xFFFFFFFFu - bits) << 32) | (uint64_t)i;
        }
        std::sort(keys.begin(), keys.end());
        int cur = -1;
        batch.clear();
        for (size_t k = 0; k < n; k++) {
            const RTri& t = tris[(uint32_t)(keys[k] & 0xFFFFFFFFu)];
            if (t.tex != cur) { flushBatch(cur == TEX_ADD ? atlasTexAdd : atlasTex); cur = t.tex; }
            batch.push_back(t.v[0]); batch.push_back(t.v[1]); batch.push_back(t.v[2]);
        }
        flushBatch(cur == TEX_ADD ? atlasTexAdd : atlasTex);
        stat = (int)(n + flat.size() / 3);
        begin();
    }
};
static Renderer R3;

// ---------------------------------------------------------------------
//  2D-интерфейс (тот же атлас, один пакет)
// ---------------------------------------------------------------------
struct UIBatch {
    std::vector<SDL_Vertex> v, vadd;
    inline void vert(float x, float y, SDL_Color c, SDL_FPoint t, bool add = false) {
        SDL_Vertex q; q.position.x = x; q.position.y = y; q.color = c; q.tex_coord = t;
        (add ? vadd : v).push_back(q);
    }
    void tri(float x0, float y0, float x1, float y1, float x2, float y2, SDL_Color c) {
        vert(x0, y0, c, UV_WHITE); vert(x1, y1, c, UV_WHITE); vert(x2, y2, c, UV_WHITE);
    }
    void quad(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, SDL_Color c) {
        tri(x0, y0, x1, y1, x2, y2, c); tri(x0, y0, x2, y2, x3, y3, c);
    }
    void rect(float x, float y, float w, float h, SDL_Color c) { quad(x, y, x + w, y, x + w, y + h, x, y + h, c); }
    void rectG(float x, float y, float w, float h, SDL_Color top, SDL_Color bot) {
        vert(x, y, top, UV_WHITE); vert(x + w, y, top, UV_WHITE); vert(x + w, y + h, bot, UV_WHITE);
        vert(x, y, top, UV_WHITE); vert(x + w, y + h, bot, UV_WHITE); vert(x, y + h, bot, UV_WHITE);
    }
    void frame(float x, float y, float w, float h, float t, SDL_Color c) {
        rect(x, y, w, t, c); rect(x, y + h - t, w, t, c); rect(x, y, t, h, c); rect(x + w - t, y, t, h, c);
    }
    void circle(float x, float y, float r, SDL_Color c, int seg = 24) {
        for (int i = 0; i < seg; i++) {
            float a0 = i * 2 * PI / seg, a1 = (i + 1) * 2 * PI / seg;
            tri(x, y, x + cosf(a0) * r, y + sinf(a0) * r, x + cosf(a1) * r, y + sinf(a1) * r, c);
        }
    }
    void ellipse(float x, float y, float rx, float ry, SDL_Color c, int seg = 24) {
        for (int i = 0; i < seg; i++) {
            float a0 = i * 2 * PI / seg, a1 = (i + 1) * 2 * PI / seg;
            tri(x, y, x + cosf(a0) * rx, y + sinf(a0) * ry, x + cosf(a1) * rx, y + sinf(a1) * ry, c);
        }
    }
    void ring(float x, float y, float r0, float r1, SDL_Color c, int seg = 32, float a0 = 0, float a1 = 2 * PI) {
        for (int i = 0; i < seg; i++) {
            float t0 = a0 + (a1 - a0) * i / seg, t1 = a0 + (a1 - a0) * (i + 1) / seg;
            quad(x + cosf(t0) * r0, y + sinf(t0) * r0, x + cosf(t0) * r1, y + sinf(t0) * r1, x + cosf(t1) * r1, y + sinf(t1) * r1, x + cosf(t1) * r0, y + sinf(t1) * r0, c);
        }
    }
    void line(float x0, float y0, float x1, float y1, float w, SDL_Color c) {
        float dx = x1 - x0, dy = y1 - y0, l = sqrtf(dx * dx + dy * dy); if (l < 1e-4f) return;
        float nx = -dy / l * w * 0.5f, ny = dx / l * w * 0.5f;
        quad(x0 + nx, y0 + ny, x1 + nx, y1 + ny, x1 - nx, y1 - ny, x0 - nx, y0 - ny, c);
    }
    void roundRect(float x, float y, float w, float h, float r, SDL_Color c) {
        r = std::min(r, std::min(w, h) * 0.5f);
        const int seg = 5;
        float cxs[4] = {x + w - r, x + w - r, x + r, x + r}, cys[4] = {y + h - r, y + r, y + r, y + h - r};
        float st[4] = {0, 1.5f * PI, PI, 0.5f * PI};
        float px[4 * (seg + 1)], py[4 * (seg + 1)]; int n = 0;
        // обход по часовой на экране: правый-нижний, правый-верхний, левый-верхний, левый-нижний
        float st2[4] = {0.5f * PI, 0, 1.5f * PI, PI};
        (void)st;
        for (int k = 0; k < 4; k++) for (int i = 0; i <= seg; i++) {
            float a = st2[k] - i * 0.5f * PI / seg;
            px[n] = cxs[k] + cosf(a) * r; py[n] = cys[k] + sinf(a) * r; n++;
        }
        float mx = x + w * 0.5f, my = y + h * 0.5f;
        for (int i = 0; i < n; i++) { int j = (i + 1) % n; tri(mx, my, px[i], py[i], px[j], py[j], c); }
    }
    void rectGH(float x, float y, float w, float h, SDL_Color l, SDL_Color r) {
        vert(x, y, l, UV_WHITE); vert(x + w, y, r, UV_WHITE); vert(x + w, y + h, r, UV_WHITE);
        vert(x, y, l, UV_WHITE); vert(x + w, y + h, r, UV_WHITE); vert(x, y + h, l, UV_WHITE);
    }
    void glow(float x, float y, float r, SDL_Color c) {
        SDL_FPoint t0 = uvp(SOFT_X + 0.5f, SOFT_Y + 0.5f), t1 = uvp(SOFT_X + SOFT_S - 0.5f, SOFT_Y + 0.5f), t2 = uvp(SOFT_X + SOFT_S - 0.5f, SOFT_Y + SOFT_S - 0.5f), t3 = uvp(SOFT_X + 0.5f, SOFT_Y + SOFT_S - 0.5f);
        vert(x - r, y - r, c, t0, true); vert(x + r, y - r, c, t1, true); vert(x + r, y + r, c, t2, true);
        vert(x - r, y - r, c, t0, true); vert(x + r, y + r, c, t2, true); vert(x - r, y + r, c, t3, true);
    }
    void softSprite(float x, float y, float r, SDL_Color c) {
        SDL_FPoint t0 = uvp(SOFT_X + 0.5f, SOFT_Y + 0.5f), t1 = uvp(SOFT_X + SOFT_S - 0.5f, SOFT_Y + 0.5f), t2 = uvp(SOFT_X + SOFT_S - 0.5f, SOFT_Y + SOFT_S - 0.5f), t3 = uvp(SOFT_X + 0.5f, SOFT_Y + SOFT_S - 0.5f);
        vert(x - r, y - r, c, t0); vert(x + r, y - r, c, t1); vert(x + r, y + r, c, t2);
        vert(x - r, y - r, c, t0); vert(x + r, y + r, c, t2); vert(x - r, y + r, c, t3);
    }
    void texQuad(float x, float y, float w, float h, float u0, float v0, float u1, float v1, SDL_Color c) {
        SDL_FPoint a = uvp(u0, v0), b = uvp(u1, v0), cc = uvp(u1, v1), d = uvp(u0, v1);
        vert(x, y, c, a); vert(x + w, y, c, b); vert(x + w, y + h, c, cc);
        vert(x, y, c, a); vert(x + w, y + h, c, cc); vert(x, y + h, c, d);
    }
    // текст: s — масштаб пикселя шрифта; align: 0 влево, 1 центр, 2 вправо
    float textW(const char* str, float s) const { int n = utf8Len(str); return n > 0 ? (n * 6 - 1) * s : 0; }
    void text(float x, float y, const char* str, float s, SDL_Color c, int align = 0, bool shadow = false) {
        if (align == 1) x -= textW(str, s) * 0.5f; else if (align == 2) x -= textW(str, s);
        if (shadow) { SDL_Color sc = RGBA(0, 0, 0, c.a * 3 / 4); text(x + s * 0.8f, y + s * 0.8f, str, s, sc, 0, false); }
        std::vector<uint32_t> cps = utf8Decode(str);
        x = floorf(x + 0.5f); y = floorf(y + 0.5f);
        for (size_t i = 0; i < cps.size(); i++) {
            if (cps[i] != ' ') {
                int g = glyphIndex(cps[i]); int gx, gy; glyphCell(g, gx, gy);
                texQuad(x, y, 5 * s, 7 * s, (float)gx, (float)gy, (float)gx + 5, (float)gy + 7, c);
            }
            x += 6 * s;
        }
    }
    // перенос по словам; возвращает высоту
    float textBox(float x, float y, float w, const char* str, float s, SDL_Color c, bool draw = true) {
        std::string cur, word; float yy = y; int maxc = (int)(w / (6 * s)); if (maxc < 4) maxc = 4;
        std::string all(str); all += ' ';
        std::vector<std::string> lines;
        for (size_t i = 0; i < all.size(); i++) {
            char ch = all[i];
            if (ch == ' ' || ch == '\n') {
                std::string test = cur.empty() ? word : cur + " " + word;
                if (utf8Len(test.c_str()) > maxc && !cur.empty()) { lines.push_back(cur); cur = word; }
                else cur = test;
                word.clear();
                if (ch == '\n') { lines.push_back(cur); cur.clear(); }
            } else word += ch;
        }
        if (!cur.empty()) lines.push_back(cur);
        for (size_t i = 0; i < lines.size(); i++) { if (draw) text(x, yy, lines[i].c_str(), s, c); yy += 9 * s; }
        return yy - y;
    }
    void flush() {
        if (!v.empty()) SDL_RenderGeometry(gRen, atlasTex, v.data(), (int)v.size(), nullptr, 0);
        if (!vadd.empty()) SDL_RenderGeometry(gRen, atlasTexAdd, vadd.data(), (int)vadd.size(), nullptr, 0);
        v.clear(); vadd.clear();
    }
};
static UIBatch UI;

// ---------------------------------------------------------------------
//  Звук: синтез в аудио-колбэке
// ---------------------------------------------------------------------
enum SfxType { SFX_BEEP, SFX_COIN, SFX_ERROR, SFX_CRASH, SFX_SHUTTER, SFX_SPEECH, SFX_RADIO, SFX_THUNK, SFX_HORN, SFX_WHISTLE, SFX_CLICK, SFX_SUCCESS, SFX_TYPE };
struct Voice { int type; float t, dur, f0, f1, vol, ph, ph2, lp, seed; bool on; };
struct Synth {
    int rate = 44100;
    // постоянные голоса
    float engRpm = 0, engLoad = 0, engVol = 0;
    float siren = 0, sirenMode = 0;   // 0 выкл, 1 вой
    float rain = 0, ambient = 0, screech = 0, wiper = 0;
    float master = 0.8f;
    bool muted = false;
    // состояние
    float ePh = 0, ePh2 = 0, eLp = 0, sPh = 0, sT = 0, nLp = 0, nLp2 = 0, scLp = 0, sirenEnv = 0;
    uint32_t noise = 22222;
    Voice vo[16];
    float nz() { noise ^= noise << 13; noise ^= noise >> 17; noise ^= noise << 5; return (noise & 0xFFFF) / 32768.0f - 1.0f; }
};
static Synth SY;
static SDL_AudioDeviceID gAudio = 0;

static void audioCallback(void*, Uint8* stream, int len) {
    Sint16* out = (Sint16*)stream; int n = len / 2;
    Synth& s = SY;
    float dt = 1.0f / s.rate;
    for (int i = 0; i < n; i++) {
        float smp = 0;
        // двигатель: пила + субгармоника, частота от оборотов
        float ef = 28 + s.engRpm * 95;
        s.ePh += ef * dt; if (s.ePh > 1) s.ePh -= 1;
        s.ePh2 += ef * 0.5f * dt; if (s.ePh2 > 1) s.ePh2 -= 1;
        float saw = s.ePh * 2 - 1, sub = s.ePh2 < 0.5f ? 1.f : -1.f;
        float eraw = saw * 0.6f + sub * 0.35f + s.nz() * (0.15f + s.engLoad * 0.25f);
        float cut = 0.05f + s.engLoad * 0.12f + s.engRpm * 0.06f;
        s.eLp += (eraw - s.eLp) * cut;
        smp += s.eLp * s.engVol * (0.22f + s.engLoad * 0.18f);
        // сирена (вой)
        float stgt = s.siren > 0.5f ? 1.f : 0.f;
        s.sirenEnv += (stgt - s.sirenEnv) * 0.0005f;
        if (s.sirenEnv > 0.001f) {
            s.sT += dt;
            float sw = 0.5f - 0.5f * cosf(s.sT * 2 * PI / 3.6f);
            float sf = 620 + 760 * sw;
            s.sPh += sf * dt; if (s.sPh > 1) s.sPh -= 1;
            float sq = s.sPh < 0.5f ? 1.f : -1.f;
            float tri = 4 * fabsf(s.sPh - 0.5f) - 1;
            smp += (tri * 0.7f + sq * 0.15f) * 0.18f * s.sirenEnv;
        }
        // дождь и фон
        float wn = s.nz();
        s.nLp += (wn - s.nLp) * 0.35f;
        s.nLp2 += (wn - s.nLp2) * 0.02f;
        smp += s.nLp * s.rain * 0.08f + s.nLp2 * s.ambient * 0.25f;
        // визг шин
        if (s.screech > 0.01f) {
            s.scLp += (wn - s.scLp) * 0.6f;
            float ff = sinf(s.sT * 2 * PI * 1900) * 0.5f;
            smp += (s.scLp * 0.3f + ff * 0.3f) * s.screech * 0.12f;
        }
        // одиночные эффекты
        for (int k = 0; k < 16; k++) {
            Voice& v = s.vo[k]; if (!v.on) continue;
            float tt = v.t / v.dur; float o = 0;
            switch (v.type) {
                case SFX_BEEP: case SFX_CLICK: case SFX_TYPE: {
                    v.ph += lerpf(v.f0, v.f1, tt) * dt; o = (v.ph - floorf(v.ph) < 0.5f ? 1.f : -1.f) * (1 - tt) * 0.5f;
                    if (v.type == SFX_CLICK || v.type == SFX_TYPE) o *= (1 - tt);
                } break;
                case SFX_COIN: case SFX_SUCCESS: {
                    float f = tt < 0.35f ? v.f0 : v.f1; if (v.type == SFX_SUCCESS && tt > 0.7f) f = v.f1 * 1.335f;
                    v.ph += f * dt; o = sinf(v.ph * 2 * PI) * (1 - tt) * 0.8f + (v.ph - floorf(v.ph) < 0.5f ? 0.15f : -0.15f) * (1 - tt);
                } break;
                case SFX_ERROR: { v.ph += v.f0 * dt; o = (v.ph - floorf(v.ph) < 0.5f ? 1.f : -1.f) * ((int)(tt * 6) % 2 == 0 ? 0.5f : 0.f); } break;
                case SFX_CRASH: { float w = s.nz(); v.lp += (w - v.lp) * 0.25f; o = v.lp * (1 - tt) * (1 - tt) * 2.2f; v.ph += 60 * dt; o += sinf(v.ph * 2 * PI) * (1 - tt) * 0.6f; } break;
                case SFX_SHUTTER: { o = s.nz() * (tt < 0.15f || (tt > 0.5f && tt < 0.6f) ? 0.8f : 0.0f); } break;
                case SFX_THUNK: { v.ph += lerpf(v.f0, v.f1, tt) * dt; o = sinf(v.ph * 2 * PI) * (1 - tt) * (1 - tt) * 1.2f + s.nz() * (1 - tt) * 0.2f * (tt < 0.1f); } break;
                case SFX_HORN: { v.ph += 370 * dt; v.ph2 += 466 * dt; o = ((v.ph - floorf(v.ph) < 0.5f ? 1 : -1) + (v.ph2 - floorf(v.ph2) < 0.5f ? 1 : -1)) * 0.22f * (tt < 0.95f ? 1 : (1 - tt) * 20); } break;
                case SFX_WHISTLE: { float f = v.f0 * (1 + 0.04f * sinf(v.t * 2 * PI * 28)); v.ph += f * dt; o = sinf(v.ph * 2 * PI) * 0.5f * (tt < 0.9f ? 1 : (1 - tt) * 10) + s.nz() * 0.05f; } break;
                case SFX_SPEECH: case SFX_RADIO: {
                    // невнятная речь: пила с «слоговой» модуляцией и формантным фильтром
                    float syl = v.t * 7.5f; int si = (int)syl; float sf = syl - si;
                    float pitch = v.f0 * (1 + 0.25f * sinf(si * 2.1f + v.seed) + 0.1f * sinf(v.t * 5));
                    v.ph += pitch * dt; float sw2 = v.ph - floorf(v.ph);
                    float env = sinf(sf * PI); if ((hash32(si + (int)v.seed * 77) & 3) == 0) env *= 0.2f;
                    float raw = (sw2 * 2 - 1);
                    float form = 0.15f + 0.25f * (0.5f + 0.5f * sinf(si * 1.7f + v.seed));
                    v.lp += (raw - v.lp) * form;
                    o = v.lp * env * 0.9f;
                    if (v.type == SFX_RADIO) { o = clampf(o * 3, -0.6f, 0.6f) + s.nz() * 0.12f; }
                    else o = clampf(o * 2.2f, -0.7f, 0.7f) + s.nz() * 0.04f * env;
                    o *= (tt > 0.95f ? (1 - tt) * 20 : 1);
                } break;
            }
            smp += o * v.vol;
            v.t += dt; if (v.t >= v.dur) v.on = false;
        }
        smp *= s.master * (s.muted ? 0 : 1);
        smp = smp / (1 + fabsf(smp) * 0.6f);
        int iv = (int)(smp * 30000); if (iv > 32767) iv = 32767; if (iv < -32768) iv = -32768;
        out[i] = (Sint16)iv;
    }
}
static void sfx(int type, float vol = 1, float f0 = 0, float f1 = 0, float dur = 0) {
    if (!gAudio) return;
    SDL_LockAudioDevice(gAudio);
    int slot = 0; float best = -1;
    for (int k = 0; k < 16; k++) { if (!SY.vo[k].on) { slot = k; best = 1e9f; break; } float rem = SY.vo[k].t / SY.vo[k].dur; if (rem > best) { best = rem; slot = k; } }
    Voice& v = SY.vo[slot];
    memset(&v, 0, sizeof(v));
    v.type = type; v.vol = vol; v.on = true; v.seed = (float)(grng.next() % 100);
    switch (type) {
        case SFX_BEEP: v.f0 = f0 ? f0 : 880; v.f1 = f1 ? f1 : v.f0; v.dur = dur ? dur : 0.09f; break;
        case SFX_CLICK: v.f0 = 1400; v.f1 = 900; v.dur = 0.035f; v.vol *= 0.5f; break;
        case SFX_TYPE: v.f0 = 2200; v.f1 = 1800; v.dur = 0.02f; v.vol *= 0.25f; break;
        case SFX_COIN: v.f0 = 988; v.f1 = 1319; v.dur = 0.35f; break;
        case SFX_SUCCESS: v.f0 = 784; v.f1 = 1047; v.dur = 0.55f; break;
        case SFX_ERROR: v.f0 = 150; v.dur = 0.45f; break;
        case SFX_CRASH: v.dur = 0.6f; break;
        case SFX_SHUTTER: v.dur = 0.12f; break;
        case SFX_THUNK: v.f0 = 140; v.f1 = 60; v.dur = 0.18f; break;
        case SFX_HORN: v.dur = dur ? dur : 0.5f; break;
        case SFX_WHISTLE: v.f0 = 2600; v.dur = dur ? dur : 0.7f; break;
        case SFX_SPEECH: v.f0 = f0 ? f0 : 140; v.dur = dur ? dur : 1.2f; break;
        case SFX_RADIO: v.f0 = 170; v.dur = dur ? dur : 1.0f; break;
    }
    SDL_UnlockAudioDevice(gAudio);
}
static void initAudio() {
    SDL_AudioSpec want, have; SDL_zero(want);
    want.freq = 44100; want.format = AUDIO_S16SYS; want.channels = 1; want.samples = 1024; want.callback = audioCallback;
    gAudio = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
    if (gAudio) { SY.rate = have.freq; SDL_PauseAudioDevice(gAudio, 0); }
}

// =====================================================================
//  Модели машин
// =====================================================================
struct MatEntry { uint8_t type, r, g, b; };
struct CarModelDesc {
    const char* name; const char* data;
    int nv0, nt0, nv1, nt1;
    float ext[3]; float fw[5]; float rw[5]; float pf, pr; float hl[3]; float tl[3];
    int nm; MatEntry mats[16];
};
enum MatType {
    M_PAINT = 0, M_GLASS = 1, M_TIRE = 2, M_RIM = 3, M_HEAD = 4, M_TAIL = 5, M_BLACK = 6, M_CHROME = 7, M_IND = 8, M_TINT = 9,
    M_BADGED = 10, M_BADGEC = 11, M_SEAM = 12, M_FRAME = 13, M_CLAD = 14, M_BADGEL = 15, M_PLATEM = 16,
    M_PAINT2 = 20, M_LBAR_R = 21, M_LBAR_B = 22, M_INTERIOR = 23, M_GLASSREAR = 24
};
// ---- Модели машин (извлечены из Mechanic3D.apk, упрощены) ----
static const char* CARDATA_SEDAN =
"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"
"hwC0AMcAAcgAkACCAAaJAMkAigAGkgBHAL0ABpMAygCUAAR+AIsAegAAfgDFAIsAAG4AUwC6AAC8AJUAbwAAvAA1AGsAAKUAwADGAACzAH8AwgAJywDHALQA"
"AMwAgADHAAGcAIAAzAABzQBmAHgAAGYAzQB5AA7OAIIAhAAGkgC9AM8ABowAkgDPAAZKAM4AhAAGjADPANAABtEAbQB7AAa0AJsAvwAAwADBANIAAdIAwQCX"
"AAHSAJcAswAB0gCzANMAAdMAswDCAAnCAKgA1AABpwCcAMwAAcsAzADHAABoANUAgQAGtQDVAGgABtYAXQBtAAbAANIA1wAA0gDTANgAANgA0wDCAAHCANQA"
"2AABpwDUAKgAAacAzADUAADUAMwA2QAA2gDbAGwABtwAeACyAACRAHkAzQAOiQCLAMkAAIwA3QBKAAawAN4AewAG3wCaAJUAANIA2ADXAADUANcA2AAA4ACQ"
"AMgABtkA1wDUAADWAG0A4QAG3QDQAOIABm0A0QDhAAbdAM4ASgAGbgC6ALgAALwAxgDjAAnLALQAvwAA5ADlAOYABucA6ADpAAbiAMQA6gAG6ADrAOkABs4A"
"yACCAAa5AOwAkwAAewDeANEABtAA3QCMAAbXAO0AwAAAiABdANYABogA7gCZAAaZAO8AsgAO1QDwAIEABs0A8QCRAA7eALAAtwAGxgDAAO0AAO4A7wCZAA7g"
"AMgA6AAGdwDyAMQABuoA6wDiAAboAN0A6wAG3QDiAOsABkEAuwC+AADGALwAawAJUAB2AMMABogA1gDuAAboAOcA4AAG6wDnAOkABmkAwwC3AAbmAPMA5AAG"
"5gDlAPMABt0AyADOAAbRAN4A5QAGzAD0APUAANcA2QD2AABaANoAbAAG1gD3AO4ABu8A3ACyAA7nAOoA8gAG4QDlANYABrUA+ADVAAbEAPIA6gAG6wDqAOcA"
"BuUA9wDWAAaKAMkAtQAG8wDlAPkABt0A6ADIAAbhANEA5QAGtwD5AN4ABsoAkwDsAAT6ALEAvgAJuwD6AL4AAHcA+wDyAAb5AOQA8wAG3gD5AOUABuQA9wDl"
"AAbcAM0AeAAA5AD5APcABrcAwwD5AAaaAPwAxQAF4wCVALwACdcA9gDtAAB2AHUA2wAGwwB2APcABsMA9wD5AAaVAP0A3wAAmgDfAPwAAPgA8ADVAAb+AMkA"
"iwAAkQDxAOwAAGwA2wB1AAbvAP8A3AAOtQDJAPgABrkAkQDsAADsAAABygAA9QDZAMwAAHYA2wD3AAbcAP8AzQAO7ADxAAEBAMUAAgGLAAADAboAlAAABAHG"
"AO0ACQUB7gD3AAYGAZAA4AAGdwCBAPsABosAAgH+AADgAAcBBgEGBgEIAVoABvAA+wCBAAbJAP4ACQEGAAHsAAEBAMwAywD0AAD1AAoB2QAJBwHgAOcABgsB"
"vwCFAAn1APQACgEJyQAJAfgABvEADAEBAQ6UAMoAAwEADQH6ALsADbgADQG7AAD2AAQB7QAAywAOAfQAAPgACQHwAAb8AAIBxQAF2wAFAfcABgwB8QDNAA78"
"AA8BAgEFEAG4ALoAABEBsQD6AAnfABIB/AAA9gDZABMBAA0BFAH6AA0EARUBxgAJBQEWARcBBssAvwAOAQkKAfQADgEABQEYARkBBgUBGQEWAQYJAfsA8AAG"
"7wDuABcBDhoBzQD/AA4bAQEBDAEOAQEbARwBALoAAwEQAQAdAcoAAAEGHQEDAcoAAPoAFAERAQ0MAc0AGgEO/AASAQ8BABEBCwGFAAkeAfIA+wAGHAEdAQAB"
"BgsBDgG/AAkXAR8B/wAOIAEeAfsABg8BIQECAQAVAeMAxgAJDwESASEBACIBIwHbAAYAAQEBHAEA3wD9ABIBAIUAsQARAQnZAAoBEwEJFQEEASQBCQkBIAH7"
"AAbyAB4B5wAGuAAlAQ0BACYBJAH2AADuAAUBFwEGGgEnARsBDv0AKAESAQAKASkBEwEJJQEUAQ0BDRoB/wAqAQ4bASsBHAEAEQEsAQsBCSkBLQETAQkrAR0B"
"HAEGlQDjAP0AAA4BLgEpAQDnAB4BBwEGLwEwASABBg4BKQEKAQADAR0BMQEAFQEyAeMACfYAJAEEAQAJAS8BIAEG7wAXAf8ADi0BMwE0AQkeATUBBwEGHwEq"
"Af8ADgwBGgEbAQ42AREBFAENJQE2ARQBDRsBJwE3AQ4SATgBIQEACQH+ADABBhYBHwEXAQ4qAScBGgEOuAA5ASUBABIBKAE4AQArAToBHQEGNgEsAREBCTkB"
"NgElAQ07ATwBKgEOMAEvAQkBBjoBMQEdAQD2ABMBJgEAIAE9AT4BBgsBLgEOAQkTATQBJgEAHgE/ATUBBkABLAE2AQkyASgB4wAAQQE8ATsBDj4BHgEgAQb+"
"AEIBMAEGGwFDASsBAAMBRAEQAQBEAQMBMQEAKAH9AOMAAC0BNAETAQkCAUUB/gAAAgEhAUUBACkBLgEtAQArAUYBOgEGEAFHAbgAACgBSAE4AQBJAT8BHgEG"
"QwEbATcBDkABNgE5AQ0FAdsAGAEGRQFCAf4AAEUBSgFCAQBDAUYBKwEAJAEyARUBCTABPQEgAQZLATQBMwEJIQE4AUUBADgBTAFFAQBNAUABOQEN2gBOASIB"
"BjoBTwExAQAyASQBUAEJFgEZAR8BBj4BSQEeAQY5AbgATQEAUQFFAUwBAEQBUgEQAQAtAS4BUwEAJgFUASQBAFUBMwEtAQnbANoAIgEGGQE7AR8BDh8BOwEq"
"AQ4/AUkBVgEGuABHAU0BACMBGAHbAAY+AVYBSQEGRAExAU8BAE0BVwFAAQ09AVgBPgEGNwFZAVoBDlsBLgELAQlQASQBVAEAKgE8AVwBDioBXAEnAQ49AV0B"
"WAEGPwFeATUBBkIBXwEwAQZZATcBJwEORQFRAUoBAJAABgFaAAY7ARkBGAEGPgFgAVYBBlwBYQEnAQ4wAWIBPQEGNwFaAUMBDkYBYwE6AQZIAUwBOAEANAFL"
"ASYBAEEBOwEYAQZKAV8BQgEAOgFjAU8BAFMBVQEtAQAiAWQBIwEGVgFgAWUBBmIBXQE9AQZDAWYBRgEAQAFXASwBDWcBUAFUAQAjAUEBGAEGQwFoAWYBACgB"
"aQFIAQAsAWoBCwEJWQEnAWEBDmsBbAFYAQZdAWsBWAEGUQFMAW0BBUgBbQFMAQA/AVYBZQEGPgFYAWABBlcBTQFuAQ1vASgBMgEAbwEyAVABCS4BWwFTAQlR"
"AXABSgEAJgFnAVQBAHEBcgFzAQZfAWIBMAEGYwFGAWYBAHQBcQEjAQYHAXUBdgEGIwFxAXMBBkMBWgFoAQBqAVsBCwEJdwFBASMBBiMBeAF3AQYjAXMBeAEG"
"cQF0AXIBBmIBawFdAQZ4AXkBegEGcwFyAXkBBnIBewF5AQYsAXwBfQEJdQEHATUBBl8BfgFiAQZ3AXgBegEGegF/AXcBBoABgQGCAQZ6AXsBgwEGewF6AXkB"
"Bm0BcAFRAQWEAUEBdwEGBwGAAQYBBoABggEGAQaFAWIBhgEAYgF+AYcBAIABBwGIAQaJAXYBdQEGgAF2AYkBBokBgQGAAQaKAWMBZgEELAFXAXwBDUcBbgFN"
"AQBQAYsBbwEJPAGMAVwBAIYBjQFgAQZiAYcBhgEAhQGGAWABBoUBjgFiAQBZAY8BWgEOgQGJAZABBoEBkQGCAQZ/AXoBgwEGaAGSAWYBAIoBTwFjAQR9AWoB"
"LAEJJgFLAZMBAEEBhAGUAQZlAV4BPwEGNQFeAXUBBpUBhQFgAQZ2AYgBBwEGdAGWAXIBBloBjwFoAQBSAUcBEAEAZwGXAVABAGABjQFlAQZ1AZgBiQEGawFi"
"AY4BBn8BgwGZAQZyAZoBewEGSAGbAW0BACgBbwFpAQCLAWkBbwEJlwGcAVABCVMBWwGdAQmeAZ8BZwEAZwEmAZ4BAGQBoAF0AQYjAWQBdAEGIgGhAWQBBtoA"
"ogFOAQZzAaMBeAEGeQGjAXMBBlgBlQFgAQZYAaQBlQEGeQF4AaMBBoABiAF2AQaBAaUBkQEGZwGfAaYBAFUBUwGdAQAmAZMBngEAhwGVAYYBAJgBdQFeAQZ/"
"AYQBdwEGpwGoAakBBpkBhAF/AQaDAXsBmgEGPAFBAaoBDkEBlAGqAQ6NAasBZQEGkQGsAa0BBoMBrgGZAQaKAWYBkgEAkAGlAYEBBpwBiwFQAQmTAUsBrwEA"
"SwEzAa8BADwBqgGMAQ5lAasBXgEGggGwAQYBBl4BqwGYAQaVAY4BhQEAhwF+AagBAIIBkQGtAQaPAZIBaAEOkAGxAbIBBkoBcAGpAQCRAaUBswEGmgGuAYMB"
"BlIBRAG0AQBuAXwBVwENtQG2AZ8BAZ8BngG1AQC3AbUBngEBtwGTAbgBAbcBngGTAQGdAbkBVQEAkwGvAbgBAboBrwEzAQAzAbkBugEAMwFVAbkBAIYBpAGN"
"AQZcAbsBYQEAhwGoAZUBAGEBvAFZAQ6oAY4BlQEAlQGkAYYBBl8BqQF+AQaCAa0BsAEGbAGkAVgBBlkBvAGPAQ5KAakBXwEAvQG+AbEBBk8BtAFEAQCdAVsB"
"vwEAnwG2AaYBAbcBtgG1AQG4AcABtwEJwQHCAboBAcEBugG5AQHCAa8BugEBWgAIAaIBBlwBjAG7AQCWAcMBcgEGvgGyAbEBBrIBpQGQAQZpAZsBSAEAnAGX"
"AcQBAJ0BxQG5AQDGAcEBuQEBwAG4Aa8BDccBwgHBAQmoAX4BqQEGiwGcAcQBAMgBpgG2AQGdAb8ByQEAxgG5AcUBAckBxQGdAQDCAccBwAENjgHKAWsBBqsB"
"vQGYAQa9AasBvgEGkQGzAawBBrMBywGsAQbDAZoBcgEGTwGKAcwBBEcBzQFuAQBpAcQBmwEAagHOAVsBAJcBZwGmAQDGAcUBzwEJwQHGAccBCc8BxQHJAQEG"
"AbABCAEGjQGkAasBBmkBiwHEAQCXAaYB0AEA0QGmAcgBAc8ByQG/AQmhAaABZAEGjAGqAbsBALwBYQHSAQ6OAagBygEGmQGuAdMBBrIB1AGlAQa0AU8BzAEE"
"0AGmAdEBAc8BvwHVAQmvAcIBwAENoQEiAU4BBrsB0gFhAQCnAcoBqAEGvAHWAY8BDmoBfQHXAQDIAbYBtwEJWwHOAb8BANUBvwHOAQHYAQgBsAEG2gBaAKIB"
"BoQBmQHZAQasAdoBrQEA2wGpAXABANwBrgGaAQa0Ad0BUgEA0AHRAd4BAcgB3wHgAQnhAZcB0AEA2AGiAQgBAKQB4gGrAQbjAeQBqwEGawHKAaQBBmwBawGk"
"AQblAacBqQEGrQHaAbABBqwBywHaAQbcAZoBwwEG1AGzAaUBBm0BmwFwAQV8AeYBfQEA5gHXAX0BAOAB5wHRAQnRAcgB4AEJ4QHQAd4BAN4B0QHnAQDfAcgB"
"6AEJtwHpAcgBCWoB1wHOAQDAAeoBtwEJ1QHOAesBAcYBzwHVAQnsAZQBhAEGqwHkAb4BBtoBywGwAQbbAeUBqQEAxAGXAeEBAOcB4AHeAQHOAdcB6wEBoQFO"
"AaABANgBsAHtAQbjAasB7gEGpAHKAeIBBpIBjwHWAQ6WAe8BwwEGmwHbAXABBd0B8AFSAQDwAUcBUgEAzQF8AW4BAOgByAHpAQnmAfEB1wEA6wHyAdUBCeoB"
"6QG3AQnYAfMBogEAqwHiAe4BBpIB1gGKAQC+AdQBsgEGzQHmAXwBAOgB4AHfAQHVAfQBxgEJ8QHrAdcBDfUB6gHAAQnAAccB9gENoAH3AXQBBtkB7AGEAQaW"
"AfgB7wEG2wH5AeUBANYBzAGKAQD6AfsBmwEF+gGbAcQBAEcB8AHNAQDhAfwBxAEA8gH9AdUBCf4B6AH/AQHpAf8B6AEB6QHqAQACCQEC9AHVAQnoAf4B4AEA"
"AgLxAeYBAMYB9AHHAQkDAusB8QEN7gHiAeMBAMoBpwHiAQanAQQC4gEGBQLWAbwBDtwBBgKuAQYHAvEBAgIACAICAuYBAekBAAL/AQHqAf8BAAIB6gEJAv8B"
"AfUBCQLqAQH1AcAB9gENAQLVAf0BCQoCAwLxAQ0LAqoBlAEOywEMArABBpsB+wHbAQUNAg4C/QEHDwIQAgECBw8CDgIQAgcPAv0BDgIH/QHyAQ0CCQ8CAQL9"
"AQnyAesBAwIJBwICAggCAQkCEQL/AQD1AfYBCQIBxwESAvYBARICxwH0AQHeAeAB4QEAEwJOAaIBAO0B8wHYAQDlARQCpwEGFQK+ARYCBhUCFwK+AQYNAhgC"
"DgIA+gHEAfwBAA0C8gEZAgcaAvQBAQIBGgIBAhACB/IBAwIZAgcRAvYBEgIAGgISAvQBAQsCuwGqAQCnARQCBAIGGwK8AdIBDgUCvAEbAg4cAgYC3AEGHQK0"
"AcwBBPkB2wH7AQUeAh8CCAIA+wH6AfwBBRkCGAINAgDmAR4CCAIAEAIOAhgCAPABHgLNAQAaAiACEgIA/gH/ASECACICGQIDAg0KAiICAwIN7AELApQBBgQC"
"4wHiAQC7ARsC0gEADALtAbABBnQB+AGWAQbTAdkBmQEG5QH5ARQCAAUCIwLWAQAWAr4B5AEGIwLMAdYBACQCrgEGAgbDASUCJgIG3AHDASYCBhcC1AG+AQYn"
"ArMB1AEGHQLdAbQBBBACGAIgAgAgAhECEgIAEQIhAv8BAPEBBwIKAgAoArsBCwIA+AF0AfcBBsMB7wElAgazAScCKQIG0wGuASQCBhwC3AEmAgbwAd0BHQIA"
"/AHhASoCAB8CBwIIAgDmAc0BHgIAGgIQAiACAAkC9gERAgBOARMCKwIAogHzARMCAKABKwL3AQDsASgCCwIA7QEMAvMBAAQCFALkAQAbAigCBQIAywEsAgwC"
"BrMBKQIsAgazASwCywEGFgItAhcCABUCFgIXAgAtAtQBFwIALgInAtQBAPsBKgIvAgD7AfwBKgIA4AEhAjACAOAB/gEhAgAxAhgCGQIAIgIxAhkCDaABTgEr"
"AgD3ASsCMgIAMgL4AfcBALsBKAIbAgDZASgC7AEAFAL5AeQBACcCLgIpAgAjAh0CzAEALQIuAtQBADMCBgIcAgbhAeABMAIAHwIxAgcCAAoCBwIxAgATAjQC"
"KwIAEwLzATQCADQCMgIrAgA0AvMBDAIABALkAeMBACgCIwIFAgA0AjUCMgIANQI0AgwCAOQBLwIWAgbkAfkBLwIALwItAhYCBikCLgI2Agb5AfsBLwIAJgIz"
"AhwCBjMCJAIGAgY3Ai4CLQIGNwI2Ai4CBhgCMQI4AgAfAh4COAIAGAI5AiACAPABHQImAgAfAjgCMQIAMQIiAgoCDdkBIwIoAgAyAu8B+AEA2QHTASMCAAwC"
"LAI1AgAsAjYCNQIAKQI2AiwCADUCNgIhAgA3AjACNgIAIAI5AjUCACYCJQIeAgAwAiECNgIANQIhAhECADgCOQIYAgAgAjUCEQIAHgLwASYCADUCOQIyAgAy"
"AjgC7wEAMgI5AjgCAO8BOAIlAgAjAtMBJAIAOAIeAiUCACMCJAIdAgAvAioCLQIAHQIkAjMCAC0CMAI3AgAqAjACLQIAHQIzAiYCACoC4QEwAgA6AjsCPAIG"
"OgI8Aj0CBjoCPQI+AgY6Aj4CPwIGOgI/AkACBjoCQAJBAgY6AkECQgIGOgJCAkMCBjoCQwJEAgY6AkQCRQIGOgJFAkYCBjoCRgJHAgZIAkkCSgIGSAJLAkwC"
"BkgCTQJLAgZIAk4CTQIGSAJPAk4CBkgCUAJPAgZIAlECUAIGSAJSAlECBkgCUwJSAgZIAlQCUwIGSAJVAlQCBkgCSgJVAgZWAlcCWAIGVgJYAlkCBlYCWQJa"
"AgZWAloCWwIGVgJbAlwCBlYCXAJdAgZWAl0CXgIGVgJeAl8CBlYCXwJgAgZWAmECYgIGVgJiAmMCBlYCYwJXAgZkAmUCZgIGZAJnAmUCBmQCaAJnAgZkAmkC"
"aAIGZAJqAmkCBmQCawJqAgZkAmwCawIGZAJtAmwCBmQCbgJtAgZkAm8CbgIGZAJwAnECBmQCZgJwAgZyAnMCdAIGcwJ1AnQCBnMCdgJ1AgZ2AncCdQIGdwJ4"
"AnkCBnkCegJ4AgZ5AnoCewIGewJ8AnoCBnsCfQJ8AgZ9An4CfAIGfQJ/An4CBn8CgAJ+AgZ/AoACgQIGgQKAAoICBoMChAKFAgaDAoYChAIGhgKHAogCBocC"
"iQKIAgaHAooCiQIGigKLAokCBooCjAKLAgaMAo0CiwIGjQKOAo8CBo0CkAKOAgaQApECkgIGkQKTApICBpEClAKTAgaUApUCkwIGlAKWApUCBpYClwKVAgaY"
"ApkClwIGmAKZAoMCBpoCmwKcAgaaAp0CmwIGnQKeApsCBp0CnwKeAgafAqACngIGnwKhAqACBqECogKgAgajAqQCpQIGpAKmAqUCBqQCpgKnAganAqYCqAIG"
"pwKpAqgCBqkCqgKoAgarAqwCrQIMrgKvArACDK4CsQKvAgyxArICswIMsgK0ArMCDLUCtgK3Agy4ArkCugIMuQK7AroCDLwCvQK+Agy9Ar8CvgIMwALBAsIC"
"DMMCxALFAgzEAsYCxQIMxALHAsYCDMcCyALGAgzHAskCyAIMygLLAswCDM0CzgLPAgzQAtEC0gIM0wLUAtUCBtQC1gLVAgbXAtgC2QIG2gLbAtwCBtoC3QLb"
"AgbeAt8C4AIG3gLhAt8CBuEC4gLfAgbjAuQC5QIG5ALmAuUCBucC6ALpAgrnAukC6gIK6wLsAu0CBusC7QLuAgbvAvAC8QIP7wLxAvICD/MC9AL1AgXzAvUC"
"9gIF9wL4AvkCBfcC+QL6AgX7AvwC/QIK/gL/AgADCgEDAgMDAwoEAwUDBgMKBwMIAwkDBgoDCwMMAwYNAwcDCAMGDgMPAxADABEDDgMQAwAIAw4DEQMAEgMR"
"AxADAA0DEQMSAwASAxADEwMADAMUAxMDBwwDEwMVAwcTAxQDEgMHDQMWAxcDBhcDFgMYAwYZAxYDDQMGGgMbAxwDBhwDGwMdAwYeAxsDGgMGHgMfAyADBiED"
"IgMjAwYkAyUDJgMAJQMkAycDACYDJQMoAwAoAyUDJwMAKQMmAygDACQDJgMpAwAkAyoDKQMAKwMsAy0DBy4DLwMrAwcwAzEDMgMPMwM0AzUDD0yRiUUF9w+T"
"WRFAN4mQ1i12OVmZ3k8oo8aQujRoqD6RUUFEvwmSCD2VVnOToD7eZcSQk0d5OzWWOxlCkQSVqReyYv6Rpj9rSbyQvi93yl+YDlXIJaCa8RB/zbGStxg8bvma"
"UFU5+ieZ9RE9o9mmRA8b9r+o9DY4p9ehW0QUivS7/jaUPXS51jZ5x9OTG0KPsISXqkenY5ybAlScEKXEmQ9jPfGk9FUENzaaNQ8Janee3hXRmNGmBzDrdG6n"
"qhJBdFqm41L2ir2yc21Rthimqg8/IQe6EzbEW7CgR1mKybi2OhByYRO3LBNEpVSg/CSSh3Od2Vff7u2ueFNfVLqw83hZ9l+9hhDfyKSi4V70J2i5JDrfuVaw"
"E3U7ChPHnjcIeRezU19rm7izu3Z+0f6mL2Tpv02yjXmN7aiqSy9crYW91VZjPD278HbWEm+3vyA7rxm+MHWquirKBySVgKHB10AxdfjM/38V8aZZbA8xz0PE"
"uRMtfD0IIV0yhcP88mGGmObANhCMZcvOETGhfpvaYhnAge/c+ExShUzJAHwgA336Dj4Lf8Q+TRAwxOHuN3wyCnQCnVeaOgf2S1XyQHkCShBtZWTgzxFsbvtP"
"oVLfWZYzWxNvfVMl3kmehGFYLA+jJY86JA8/PRED/TQPf2001XsvBpFCy3asEqgnVHj2ucRJ5BXppIMw2DBifu4uvxeFg5U+81RfPoUoPRYYlQQ2K35L8WdH"
"dxCzYpEZxBG6bnA1GSRpgDk+1UAKdWJIZ3Pnuy0uaCVUjU5a01ZiN/g2IzfEeOpa9i5mdOpSkRUDd8VaPFMcjOpO5F6Wm9ddKCStiBNIMxvghBxETDUDXGRC"
"4DO8PCNiMEQAjNFLpBwXiZ5EWRJTkJlDHzRqthVFvz5uwM5lqTxNa4dMV3mC7XtT4hjxjoNMXnS1zuZtzkMCTPlo9hNeaudXRipzysxQTDMUsFBQUXPPDtRq"
"eUXfYcdhiFf27sxN63fQ9ZBmbzUTqEJm31CNo1Jl9VTF+VJrpT9ewPRtiSDlzftaOmC2vRRrTg+JNchb3FxdKWtuqizlX4NfzVcXzVNkAhWSmYdoCFT3J35s"
"jCCKo3Nk/VIIIdZrVRDkYE5vT0UT/FJqzg5c4UJvojE1Oi5v1j68sP1plytCV/1pdhN1WP1piwVZSf1pKB1PP/1pBjKYRvCVlytCV/CViwVZSfCVdhN1WPCV"
"KB1PP/CVBjKYRv1pqifeq/1pBzK7vv1pKB0Exv1pSgi8vv1prguKsPCVqifeq/CVKB0ExvCVBzK7vvCVSgi8vvCVrguKsNMfXUHvgyzgXUF5gyHgxEuUhMgf"
"xEsKhSYckEO4g9fjkENPg9HjoUn0gx8coUldhAAAAQACAAADAAQABQAABgAHAAgAAAQAAwAJAAAHAAYACgAAAgALAAgAAAUADAAAAAACAAgADQAACwAGAAgA"
"AAAAAgANAAAAAAwADgAADwAHAAoAAAAADQAQAAAEAAkAEQAAAQAAAA4AAAEADgASAAATAAQAEQAGFAAJAAMAAAIAFQALAAYMAAUAFgAGFwAFAAQAAAcAGAAI"
"AAAZABAADQAAGgACAAEABgwAFgAOAAYbAA0ACAAAHAAPAAoAAAkAHQARAAAHAA8AHgAAHAAfAA8AAAIAGgAVAAYXAAQAEwAGIAAUAAMABQMABQAhAAABABIA"
"IgAAIwAKAAYABgUAEAAkAAAFAAAAEAAAAQAiABoABiUAHAAKAAYRACYAEwAGFAAnAAkAAAsAIwAGAAYoACQAEAABCAAYACkAACoAEAAZAAkWACsADgAGLAAZ"
"AA0ACSQAIQAFAAAXAC0ABQAGGwAIACkAABkALgAqAAkmABEAHQAGLwAYAAcABCAAAwAwAAAHAB4ALwAAFwATAC0ABhAAKgAoAA0xADIAJAAJEgAOACsABgoA"
"IwAlAAYLABUAIwAGGwAsAA0AACEAJAAyAAAzACQAKAAJLAAuABkACSoAMwAoAA0lAB8AHAAONAATACYABgUALQAWAAYbACkANQAAGwA2ACwAACQAMwAxAAk3"
"ADQAJgAGMgA4ACEAACcAOQAJAAYvADoAGAAEMAADACEAADIAMQA4AAAtABMANAAGLgAsADYAADMAKgAuAAEzAC4AOwAAPAAaACIABj0ADwAfAAAeAA8APQAO"
"JwAUADkAAD4AIAAwAAAzADsAMQAAFgAtADcABjAAOAA/AAk7ADgAMQAAJQBAAB8ABj0AQQAeAA5AAD0AHwAOKwAWADcABjgAMAAhAAkjABUAGgAGEgA8ACIA"
"BisANwAmAAYjABoAJQAGCQBCAB0ABiAAQwAUAAUgAD4AQwAAHgBBAC8AAAkAOQBCAAZEADsALgAARQApABgAABQAQwA5AABGADwAEgAGOgAvAEEAAC4ANgBE"
"AABEAEcAOwAJSAA2ABsACRgAOgBFAAApAEkANQAASAAbADUACSUASgBLAAZHAEQANgAAQgAmAB0ABj0AQABLAA4pAEUATAAAQQA9AE0ADksASgA9AA5DAD4A"
"TgAATwBQABoABkAAJQBLAAZBAFEAOgAARwBSADsACVEARQA6AAYwAD8APgAANgBTAEcAACsAJgBGAAY7AFQAOAAAJgBVAEYABkoATQA9AA5JAEgANQANQQBN"
"AFYADkIAOQBXAAYpAFgASQAAWABIAEkADUIAVwBZAAZIAFMANgAJOwBaAFQAACYAWQBVAAZbAE0AXAAOWQAmAEIABjkAXQBXAAZBAFYAUQAAUgBaADsACUMA"
"TgA5AABHAFMAUgAAUQBeAEUABiUAGgBKAAZOAF0AOQAAVgBeAFEAAFQAPwA4AAk/AFQAXwAJGgA8AE8ABkoAXABNAA4pAEwAWAAAUABKABoABkwARQBeAABX"
"AGAAWQAGYQBTAEgACSsARgASAAZbAFwASgAGVgBiAF4AAEgAWABhAA1QAFsASgAGVgBjAGIAAFYATQBkAA4+AGUATgAAZgA+AD8AAGYAPwBfAAlXAGcAYAAG"
"XQBoAFcABlsAUABpAAZqAGkAUAAGZQBrAE4ABWwAVwBtAABuAEYAVQAGbgBvAEYABnAAXgBiAARXAGgAbQAAbABtAFkABlQAWgBxAABtAFUAWQAGcgBsAFkA"
"BmcAVwBsAAZfAFQAcwAAYAByAFkABlQAcQBzAABoAHIAbQAAaQBqAHQABk0AWwB1AA5wAGIAYwAATQB1AGQADkYAbwB2AAZuAHcAbwAGTgBrAGcAAFMAeABS"
"AABaAHgAcQAAWgBSAHgAAGgAbAByAABkAHUAVgAOXQBnAGgABkYAdgA8AAZnAHIAYAAGVgB1AGMADk4AZwBdAABeAHkATAAAUwBhAHgAAHEAegBzAAl7AHEA"
"eAABfABvAHcABmYAZQA+AABmAF8AfQAAbABoAGcABnsAfgB6AA13AFUAfAAGbwB/AHYABn8AgAB2AAZmAH0AZQAAbQByAFUABl8AcwCBAABPAIIAUAAGeQBe"
"AHAABHEAewB6AA1hAIMAeAAAWwBpAIQABnMAhQCBAAlyAIYAVQAGhwCDAGEAAFUAiAB8AAZ2AIAAPAAGfQBfAIEAAIgAVQCGAAZyAGcAhgAGTABhAFgAAIUA"
"cwB6AAmDAIkAeAAJfAB/AG8ABkwAhwBhAAB4AH4AewAJhACKAFsABnoAfgCFAAF+AHgAiQAJhwCJAIMADWUAfQBrAAV+AIsAhQAAjABPADwAAIAAjAA8AAB5"
"AIcATAAAigB1AFsABlAAjQBqAAZnAGsAhgAAdQB5AGMAAHkAcABjAABqAI0AdAAGfgCJAIsAAIsAjgCFAACNAFAAggAGhABpAHQABk8AjACCAACOAH8AfAAA"
"gQCFAI4AAIQAdQCKAACGAGsAiAAAjACLAIIAAIsAjACAAACIAI4AfAAGiABrAI4AAGsAfQCOAACJAIcAjQAAiQCNAIsAAIQAeQB1AACAAH8AiwAAiwB/AI4A"
"AHQAjQCHAACHAHkAdAAAiwCNAIIAAHkAhAB0AAB9AIEAjgAAjwCQAJEABo8AkQCSAAaPAJIAkwAGlACVAJYABpQAlwCVAAaUAJgAlwAGmQCaAJsABpkAmwCc"
"AAaZAJwAnQAGngCfAKAABp4AoQCfAAaeAKIAoQAGowCkAKUACqMApQCmAAqnAKgAqQAGpwCpAKoABg=="
;
static const char* CARDATA_HATCH =
"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"
"CQAoAAATACkAFgAADwAKACoAACsAEwAlAAAIACUABgAACQAVAAoAACsAJQAIAAATACsALAAAKwAtACwAABAADwAqAAArAC4ALwAAAgAFAAMADQUAMAADAA0d"
"AB4AHAAAMQAyADMAABkANAAaAAAZABsABwAACwAOABwAAAsAHAAfAAAZAAcABgAABwAOAAgAABcANAAGAAAgABQANQAANgAkACMAACcAEQAkAAA3ADgAFAAA"
"IwARABYAABQAIgA5AAAVABQAOQAAKgAuABAAACsACAAuAAAIABAALgAAEwAsACkAACwALQA6AAAwAAQAAwAAOwA8AD0AAD4AGgA0AAAzADIAPwAAPwAgADMA"
"ABkABgA0AAAOAAcAGwAAIAA/ACEAAA0ACwAfAABAABcAJgAAJwAmABIAADEAQQAyAABCADEAMwAAIABDADMAADUAQwAgAAAhAEQAIgAAKAAJAAwAACsALwBF"
"AABGAAAABAAARgABAAAAAEcASAAYAAAYAEgAGwAASABJABsAABsASgAdAABAAEsAFwAATAAMAA0AADUAFAA4AAAoADcAFAAAFgBNAE4AACkATQAWAAA6ACkA"
"LAAAGABPAEcAAFAAPgA0AAANAB8ATAAAHwBRAEwAAFAANABLAAA0ABcASwAATABSAAwABiYAUwBAAAYoAFQANwAGCgAEACoAADAAKgAEAAABAEYABQAAHgBV"
"AFYAAFcAJAA2AAYnAFgAJgAGJgBYAFkABloAOQAiAAAwAFsAKgAABQBcADAADV0ASABHAABJAEoAGwAASgBJAF4AAEoAXwAdAABgAGEAPwAAYQAhAD8AAEsA"
"YgBQAAY1AGMAQwAGLQArAEUAAEUALwBkAAAvAC4AZQAALwBlAGYAAEYAXAAFAABIAF0AZwAGSQBIAGcABmgAPwAyAAAzAGkAQgAAQwBpADMAAEAAagBLAAYh"
"AGEARAAADABSAGsABmwAagBAAAYiAEQAbQAAWgAiAG0AABUAOQBuAAAwAG8AWwAACgBGAAQAAAoAFQBGAABwAGUALgAAXgBfAEoABkEAaAAyAAY/AGgAYAAA"
"VgBRAB8AADUAOABjAAYkAHEAcgAGOgBzACkAAC4AKgBbAABbAHAALgAAMABcAG8ADXQAXABGAABdAEcAdQAGPAB2AD0AAHcAPAA7AAB3AHgAPAAAHwAeAFYA"
"AEwAUQB5AAYMAGsAegAGWQBTACYABloAewA5AAcVAG4AfAAAcwBNACkAAG8AdABbAAlwAFsAfQAJfQBlAHAAAEUAZAB+AAllAH8AZgAAgAB2ADwAAFAAgQA+"
"AAZAAFMAbAAGKAAMAHoABigAegBUAAZEAIIAbQAATgBNAIMAAHsAhAA5AAc5AIQAbgAHhQAVAHwAAIUARgAVAACGAEYAhQAARQB+AIcACWQALwBmAACGAHQA"
"RgAAZwCIAEkABl4ASQCJAAZLAGoAYgAGJAByAFgABm0AewBaAAeKAG4AhAAHLQCLAIwACToALQCMAAk6AIwAcwAAZABmAH4ACX0AfwBlAABcAHQAbwAJWwCN"
"AI4ACV0AdQBnAAZeAIkAXwAGGgA+AI8AAD4AgQCPAAaQAIQAewAHbgCRAHwAAJEAbgCKAAd/AH4AZgAAWwB0AJIACUEAkwCUAAYxAJUAQQAGlgBtAIIAAJcA"
"ewBtAASYAJAAewAHhACQAJkABJkAigCEAAB8AJoAhQAAjACbAHMAAIcALQBFAAmHAIsALQAJhgCSAHQAAJwAYQBgAACXAJgAewAEfACdAJoAAE0AngCDAABN"
"AHMAngAAWwCSAI0ACXUARwBPAAafAFYAVQAGQwCgAGkABpAAmAChAAeZAJAAoQAEogCRAIoAB4oAmQCiAACMAIsAmwAJfQBbAI4ACZIAowCNAAB/AKQAfgAA"
"aABBAJQABmgAlABgAABgAJQAnAAATAB5AFIABmoApQCmAAaWAJcAbQAEdQCnAGcABkQAYQCoAACCAJcAlgAEmACXAKkABKMAkgCGAACbAIsAqgAAfQCOAKsA"
"CaMArACNAACrAK0AfgAApACrAH4AAH8AfQCrAAmuAFYAnwAGVwCvAHEABqYApQBYAAZ6ALAAVAAGqQChAJgAB7EAkQCiAAeyAIUAmgAJcwCbALMAAJsAqgCz"
"AACkAH8AqwAJgAC0AHYAAHYAtAB1AAaIAIkASQAGdwC1ALYAALcAuAC1AAZ3ALcAtQAGjwC4ALcABp8AuQCgAAagALkAaQAGQwBjAKAABroAYwC7AAYkAFcA"
"cQAGOAC7AGMABjgANwCwAAYnACQAWAAGvACCAEQAAJkAsQCiAAd8AJEAnQAAkQCxAL0AAIYAvgCjAAmeAHMAswAAjQCsAL8AAIcAfgDAAAl+AK0AwAAJwQCr"
"AI4AAI8AgQC4AAZWAK4AeQAGVgB5AFEABlAAYgCBAAawALsAOAAGcgDCAFgABqkAsQCZAAeZAKEAqQAHiwCHAKoAAKsAwwCtAACgAGMAugAGRACoALwAALAA"
"NwBUAAa8AMQAggAAnQCRAL0AAIMAxQDGAACFALIAhgAJgwCeAMcAAMEAjgDIAACOAI0AyAAAwwDAAK0AAMEAwwCrAAB4AHcAtgAAYgBqAKYABsIApgBYAAbF"
"AIMAxwAAqQDJALEAB5oAnQCyAADBAMgAygAAuAC2ALUABpcAggDLAATMAKkAlwAEwACqAIcAAK4AnwCgAAa4AIEAtgAGcQCvAM0ABnEAzQByAAZiAKYAwgAG"
"vACoAMQABssAzACXAASUAM4AnAAAwgC2AIEABlIAeQBrAAZiAMIAgQAGzQC2AMIABnIAzQDCAAbJAKkAzAAHsQDJAL0AAMMAwQDKAADKAM8AwwAAdQC0AIAA"
"Bq4AoAB5AAa6AGsAoAAGawB5AKAABmEA0ADRAACwALoAuwAGugCwAHoABmsAugB6AAaCANIAywAAjQC/AMgAAMAAwwDPAACIAJUAiQAGYQDRAKgAAKgA0QDE"
"AAazAMcAngAJqgDTALMACb0AyQDMAAC9ANQAnQAAsgDVAIYACdYAzgCUAADXAKoAwAAAQQCVAJMABs4A2ACcAACsAKMAvwAJzwDXAMAAAGEA2QDQAAC9ANoA"
"1AAA1wDTAKoAAIIAxADSAADMANIA2wAAvQDbANoAAIYA1QC+AAnEANEA3AAGnQDUALIADcsA0gDMAADHALMA3QAJpwDeAGcABmEAnADZAADKAMgA3wAA1ADV"
"ALIADdMA1wDgAACUAJMA1gAGvQDMANsAAIAA4QB1AAajAOIAvwAJ4wDVANQADdcA5ADgAADlANgAzgAAxwDdAMUACdcAzwDmAADRAOcA3AAGxADcANIAANIA"
"6ADbAADTAN0AswAJ2ADZAJwAAN0A0wDpAAnpANMA4AAJyAC/AOIAANYA5QDOAACjAL4A4gAJ6gDrANAAAOwA7QDaAADUANoA7QAA0QDQAOsAANEA6wDnAAbS"
"ANwA6AAA1ADtAOMADdcA5gDkAADrAO4A5wAG2QDqANAABucA7wDcAADwAMoA8QAA6wDqAO4AAPIA6QDgAAnKAN8A8QAAzwDKAPAAAPMA5QDWAADpAPQA3QAJ"
"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"
"AYoBawEJfQF/AUoBCYABcwFOAQkrARoBdwEGKwF3AYsBBogBgwFpAQBiAXwBFQEAfAGMAY0BACsBjgGPAQaQAW0BWQEANwEyAWUBBmQBZwGRAQZdAZIBXgEG"
"kwGHAVEBAIgBegGGAQeIAYIBegEHiQGDAYgBB2IBjAF8AQCAAZQBcwEJfQGVAX8BCW8BjQGWAQlvAXwBjQEAlwGYASQBAB0BGwGZAQaaAWMBUwEAVwGbAVQB"
"BjUBdAFaAQZmAWUBnAEGdAF5AZ0BBnkBbgGdAQacAV8BZgEGhgGHAZMBB54BhgGTAQeeAYgBhgEHhQFrAYoBCX8BlAGAAQBxAZ8BZQEGoAF4AUgBBokBiAGe"
"AQeNAaEBlgEJogGjAY0BCXMBlAGKAQCjAaEBjQEJlAF/AZUBAI0BpAGiAQCOASsBiwEGHQGlAXABBkYBmgEtAQCOAaYBjwEAWQFwAZABAKcBUQFtAQBfAZwB"
"qAEGewGJAakBAH0BlgGqAQChAasBlgEAfQFvAZYBCaQBjQGMAQAkAZgB3gAGNAE1AVcBBmQBkQGsAQatAV0BWwEGoQGjAasBAJUBfQGqAQDeAIsBdwEAiwHe"
"AKYBAKUBHQGZAQalAZABcAEAVwGuAZsBAGQBrAFYAQanAZMBUQEAqAGdAW4BBnIBrwGwAQBiAXsBqQEAlgGrAbEBAJQBlQGqAQCyAaQBjAEAswFsAYEBAJ8B"
"tAGcAQZlAZ8BnAEGoAG1AXgBBlsBeAG1AQZaAXQBnQEGkgFdAbYBBrYBkQGSAQZdAa0BtgEGXwGoAW4BBq8BcgGEAQCFAYoBtwEAjAGrAbIBAIwBuAGrAQCk"
"AbkBogEAWgGuAVcBAKwBkQG2AQaeAboBiQEAdQFyAbABAIQBhQG3AQC7AaQBsgEAuQGkAbsBAJgBlwHeAACLAaYBjgEAVQG8AVYBAGwBswG9AQBxAWwBvQEA"
"vQG+AXEBAFgBvwFVAQCfAXEBvgEAnwG+AbQBALUBoAHAAQC1Aa0BWwEGtAHBAZwBBsEBqAGcAQapAYwBYgEAuAHCAcMBAMIBuAGMAQCqAZYBsQEAlwGmAd4A"
"AK4BmQGbAQDEAcUBvQEAWgHGAa4BAKwBxwFYAQCsAbYBxwEArQG1AcABAK0ByAG2AQDAAcgBrQEAqQHCAYwBAKMByQGrAQClAZkBrgEARgFjAZoBAJABpQGu"
"AQC/AcoBxAEAygHFAcQBAFgBygG/AQCQAa4BxgEAkAGnAW0BAMsBwAGgAQDMAcEBtAEAnQHGAVoBAMwBqAHBAQC6AZMBpwEArwGEAbcBAKoBzQGUAQCiAc4B"
"yQENlwGzAaYBALwBVQG/AQC/AcQBvAEAvQHFAb4BAMcBzwHKAQBYAccBygEAxgGnAZABALQBvgHDAQCvAcABsAEAwAHLAbABAJMBugGeAQC6AcwBqQEAugGp"
"AYkBAM0BigGUAQCjAaIByQENswGXAbwBAMQBswG8AQCzAcQBvQEAygHPAcUBANAB0QHFAQC+AcUB0QEAxgG6AacBAJ0BugHGAQDMAbQBwwEAxwG2AcgBAMMB"
"wgHMAQDAAa8ByAEAnQGoAcwBAJ0BzAG6AQDMAcIBqQEA0QGqAbEBALcBzQHSAQCxAasBuAEAtwHSAa8BALgB0QGxAQDNAbcBigEAogG5Ac4BDb4B0QHDAQDF"
"Ac8B0AEAyAHSAccBANIBzwHHAQDPAdIBzQEArwHSAcgBANEB0AGqAQDNAdABzwEAuAHDAdEBANABzQGqAQDJAbsBsgEAuwHOAbkBAKsByQGyAQC7AckBzgEA"
"0wHUAdUBBtMB1QHWAQbTAdcB2AEG0wHYAdkBBtMB2QHaAQbTAdoB2wEG0wHbAdwBBtMB3AHdAQbTAd0B3gEG0wHeAdQBBt8B4AHhAQbfAeIB4AEG3wHjAeIB"
"Bt8B5AHjAQbfAeUB5AEG3wHmAeUBBt8B5wHmAQbfAegB5wEG3wHpAeoBBt8B4QHpAQbrAewB7QEG6wHtAe4BBusB7gHvAQbrAe8B8AEG6wHwAfEBBusB8QHy"
"AQbrAfIB8wEG6wHzAfQBBusB9QH2AQbrAfYB7AEG9wH4AfkBBvcB+gH4AQb3AfsB+gEG9wH8AfsBBvcB/QH8AQb3Af4B/QEG9wH/Af4BBvcBAAL/AQb3AQEC"
"AgIG9wH5AQECBgMCBAIFAgcGAgcCCAIHBQIHAgkCBwoCCwIMAgALAgoCDQIADgIPAhACBRECEAIPAgUPAhICEQIFDgITAg8CBRQCFQISAgURAhICFQIFDgIW"
"AhcCBRgCEwIOAgUXAhgCDgIFFgINAhcCBhkCCAIaAgYVAhsCCAIGFQIIAhkCBhACEQIZAgYOAhYCDQIGHAIdAh4CBx0CHwIeAgcgAh4CIQIHHwIhAh4CBx8C"
"IgIjAgcdAiICHwIHJAIhAiUCByQCJQIjAgcmAicCKAIAKQIqAisCACsCLAItAgAqAiwCKwIALgIvAjACBS8CMQIyAgUvAjICMAIFLwIcAjMCBRwCNAIzAgU0"
"AhwCNQIFHAIvAi4CBTICNgI3AgU2AjICMQIFJgIpAisCBjUCHgIcAgYuAigCHAIGLgImAigCBi4CJgIwAgYmAjACKQIGMgIpAjcCBjgCOQI6AgY7AjgCPAIA"
"PAI6Aj0CBjgCOgI8AgA6AjkCPQIGOwI+AjgCAD0CPwJAAgZBAkICQwIGQwJCAkQCBkUCRgJHAgZIAkcCRgIGRQJCAkgCBkECRQJHAgZBAkMCSAIGQQJIAkYC"
"BkICRwJIAgZEAkECRgIARAJGAkUCAEMCRQJEAgZDAkUCSAIGDwJJAhICAEoCFgJLAgAPAhMCSQIATAJJAhMCAEoCEwIYAgAWAkoCFwIAGAIXAkoCAE0CNwJO"
"AgAvAk8CMQIAMQJPAjYCAE4CNgJPAgA3AjYCTgIAUAJRAlICCVMCUgJRAglSAlQCUAIJVQJQAlQCCVYCVwJYAglZAloCVwIJWgJYAlcCCVgCVQJWAglVAlQC"
"WwIJWwJcAlUCCVMCUQJdAglWAlUCXAIJFgJeAksCAF4CXwJgAgBeAhYCDgIAYAJhAlwCAFwCWwJgAgBbAlQCYAIAYQJgAl8CAGICYwJkAgBjAmICXwIAVAJS"
"AmACAFICUwJLAgBLAmACUgIAYAJLAl4CAF8CXgJjAgBTAl0CSwIAEQIVAmUCABACEQJjAgAOAhACYwIAZAJjAmYCAGQCZgJnAgBoAmYCYwIAZwJpAmYCAGgC"
"ZgJpAgBeAg4CYwIAZQJjAhECADcCagIyAgBrAl8CYgIAVgJcAmECAGsCZwJfAgBfAmoCYQIAVwJWAmECAGoCXwJnAgBZAlcCYQIAYQJNAlkCADACMgJsAgBt"
"AlkCTQIATQJhAmoCADUCHAJuAgBqAmcCbAIAagI3Ak0CAC4CMAJsAgBnAm4CbAIAZwJrAmQCAG8CZwJwAgBnAm8CbgIAagJsAjICAG4CLgJsAgAuAm4CHAIA"
"cQJjAmUCAGMCcQJoAgBpAnACZwIAcgJpAmgCBmkCcwJwAgZpAnQCcwIAaQJ1AnICBnECZQIVAgZ2AncCdAIFdAJ3AngCBXQCeQJ2AgV6AnsCfAIAfQJ+AnoC"
"AH4CewJ6AgAUAhICfwIAfwJ7AoACAH4CgAJ7AgCAAn4CgQIAFAJ/AhUCAHkCdgKBAgBxAoICFQIAgwKCAhUCAIQCgwIVAgBpAngChQIAeAJ1AoUCAHUChgKF"
"AgCGAmgChQIAhwKFAmgCAIIChwJoAgBoAnECggIAeAJ1AoECAHUChgKBAgCBAoYCgAIAcgKAAoYCAGgCgAJyAgBoAnECgAIAgAJxAhUCAIACFQJ/AgB8AogC"
"egIAiQJ6AogCAHwCigKLAgCMAooCTAIAiwKKAowCAHwCiwKIAgCIAosCjAIAiAKNAokCAIoCfAJ7AgB6Ao4CfQIAjwJ9Ao4CAIkCjgJ6AgCQAogCkQIAiAKQ"
"Ao0CAJICjAKTAgCMApICiAIAkQKIApICAEwCigJJAgB/AooCewIAfgJ9ApQCAHkClAKVAgCUAnkCfgIANAKVAjMCAG8ClgJuAgA1Am4ClgIAlgKXAjUCAHQC"
"eAKYAgBvAnMCmAIAmAJ0AnkCAJgCeQJzAgBzAnkCcAIAcAJ5Am8CAG4CbwJ5AgCVAm4CeQIAbgKVAjUCADUClQI0AgCZApoCmwIAmwKaApwCAJwCmgKdAgCe"
"ApkCnwIAmgKZAp4CAJoCfQKgAgCaAqACnQIAoQKfApkCAJkCmwKhAgCeAp8CLwIAmgKeApQCAH0CmgKUAgCVAi8CMwIAfQKiAqACAKMCoAKiAgCgAqMCnQIA"
"pAKbAqUCAJwCpQKbAgCbAqYCoQIApgKbAqQCAJ4CLwKVAgCUAp4ClQIAhQKYAmkCAHgCaQKYAgB+AnkCgQIAogJ9Ao8CAIoCfwJJAgBJAn8CEgIApwKoAqkC"
"BqoCqAKnAgarAqwCrQIGrgKrAq8CBq0CrwKrAgawArECsgIGsQKzArQCBrQCsgKxAgZbArUCXAIGtQJbArYCBrYCUgK3Aga3Al0CuAIGuQJdArcCBlICtgJb"
"Aga6Am0CuwIAvAK6AlcCBrwCvQK1Aga8AlcCvQIGXAK1Ar0CBroCuwK8AgC+Ar8CwAIGvwK+AsECBsICwwLEAgbFAsQCwwIGxQLGAsQCBsMCxQLHAgDGAsUC"
"xwIGxwLDAsICBsgCyQLKAgDKAssCzAIAyQLLAsgCAMgCygLMAgDJAssCzAIAzQLOAs8CAM4C0ALRAgDRAs8CzgIASgJMAhMCAE8CLwKfAgBtAloCWQIJ0gLT"
"AtQCBtQC1QLSAganAqkC1gIG1wKnAtYCBqoCpwLXAgaqAtcC2AIG1gKoAtkCBqkCqALWAgasAtoCrQIG2wKtAtoCBq8CrQLcAgbcAq0C2wIG3QKwArICBrIC"
"3gLdAgayArQC3wIG3gKyAt8CBuACuAK2Aga1ArYCuAIGtQK4ArsCBuEC4gK7Aga1ArsC4gIG4wLkAuUCBuUC5ALmAgblAucC5gIG5wLmAugCBucC6QLoAgbp"
"AugC6gIG6QLqAusCBusC6gLsAgbrAuwC7QIG7QLsAu4CBu8C8ALxAgbxAvAC8gIG8QLyAvMCBvMC8gL0Agb1AvYC9wIG9QL4AvYCBvgC+QL2Agb5AvoC+wIG"
"+gL8AvsCBvoC/QL8Agb6Av4C/QIG/gL/AgADBv4CAQP/AgYBAwID/wIGAgMDAwQDBgMDBQMEAwYDAwYDBQMGBgMHAwUDBgYDBwP1AgYIAwkDCgMGCQMLAwwD"
"BgwDDQMLAwYMAw4DDQMGDgMPAw0DBg4DEAMPAwYOAxEDEAMGEQMSAxADBhEDEgMTAwYTAxIDFAMGEwMVAxQDBhUDFgMUAwYVAxcDFgMGFwMYAxYDBhcDGQMY"
"AwYaAxsDGQMGHAMdAx4DBh4DHwMdAwYfAyADIQMGIQMgAyIDBiEDIgMjAwYjAyIDJAMGIwMkAyUDBiUDJAMmAwYlAycDJgMGJwMoAyYDBicDKAMpAwYpAygD"
"KgMGKQMqAysDBisDKgMsAwYtAy4DLwMGLgMwAy8DBi4DMQMwAwYxAzIDMAMGMQMzAzIDBjMDNAMyAwY1AzYDNwMGNgM4AzcDBjYDOQM4AwY5AzoDOwMGOQM8"
"AzoDBjwDPQM6AwY8Az0DPgMGPgM/Az0DBj4DPwNAAwZAA0EDPwMGQgNDA0QDBkQDRQNDAwZFA0YDQwMGRwNIA0kDBkkDSgNIAwZJA0sDSgMGSwNMA0oDBksD"
"TANNAwZNA04DTAMGTQNPA04DBlADUQNSAwpQA1MDUQMKVANVA1YDClQDVgNXAwpYA1kDWgMKWANaA1sDClwDXQNeAwpcA14DXwMKYANhA2IDD1OXTSG8Z9Cv"
"cDCReIKUKzRtXSuWwiGVkb6UVjgoo5GOzT6esBmP0TanE8uRnjmsR9ia61E5JZig8FRa3+SbKg6YvW+a8A5TO/CZYEJZkY6jpkuUU7uq3A4kdnmY6BALZSei"
"TVWCthioO1g0NUegVFssJhmZZxs2wZGY/RouNLy9dDLYPuK8DTRkXhGuUBKIO+mhBUKSZ/eeqFLj8zau2Q5nvJGmpzaxsN6w31/hnE+0q1JuQu+uM3GL4WWy"
"O3UJ7p+yXDkIc9OpkkoxXWqw+3FjxCiu3Q4lPFC7aDJst424yyWzjwOoeGA4r92rVm3gEPau9RGJvSzAZ0ffZ76vpQ+nZXm52XLIFWa9TRPvNXa8P3WnrnvD"
"F1EiSeX+03bgF83Ke31o5RXAk1SUkUC5PhFmkRauJRDplCiuDQ+BYzQDJTXmfI79WV73kLhS4w6RvC9gzw7zvMzfWihRfoFArg58kcIYQ1JKRPNZBxDOcywP"
"ahr4fdYf/3+n14Pf6w5DeykGD1GkTecCky/qfW5ADUZTbftF8HRqrugvzg2betQbKSnZfpxS8S8TeLdKPUp5X+JTczhicvFA71AHSdBIm3IeFqlXMVOKPL1S"
"4w4jPLhS9Q5HY2plrxCvZdxOcHNI4fhMeHRRz7c631XEkc5BYxNKkXNBQDcWrp5BvRNYwtxIayPWkMtBmBMBNihCyzKbYO1BCRMRaGhCoTFpPvVN0VndmFJS"
"FG1AEsFMk3S28ctSJhKAO41ShBLakrZSGBIwZIJs5xqBwbxSdhIAvU5q1jZ4WxVZMTdYqrht8zAat+ltdS7FO/Zp7xEXOsNRZWrAttFcpVXXrz1v1j4/kVlh"
"5A0QO0xqYDQ+nZpTrGrZ661bHEjjV2lgqVSm4TRgV1IZ859l8U5BLGxeXlmoJiZqpCJGkclhslDjIzB0m1XxJf9wfDh4WYJwhBAOvRpr9g/iXBprPycIRBpr"
"9SXzWRprHAnRRzKVPycIRDKV9g/iXDKV9SXzWTKVHAnRRxpr9g8WmxprWSw9pBprUBnftxprCghDrDKVUBkFmTKV9g/OtTKVWSynrDKVCgiipABRm0TRjK5f"
"8kYqloZiezEdh5JHzC/WhhizZDgkjDajjDH3iVSu40PcjW2d4kY1lRtAvDfJhgU/ajn8hig/8DjZhm8+8jemhpU9eDd0hg9ABDhrhvJASjfGhg8/izmDhg5H"
"VFddlbG24FMWlV7tjGFZlRUryVNLkB/54FPai3UCO0PqhV8B9C58hbLfoTG0hD3k7y57hZMaui50hX7luS5+hcP8ti76hFcIuS50hasBQi54gNcGZy4JhVEf"
"5hV3ghxjYhQYkMACuC7DhqwLuS52hXdtsyvKlS8VphRfipOcgiHRhE7ksS7Ehi6e+xKFj32TFyuKlY9kyhTyjbdu7C2nlA1BuC0Dlc1ADxTxlJWR7C2nlLyb"
"yhTyjT+/uC0DlX+/DxTxlPQBrU6Lien//06IiQICwE6PiQUFBk8BiqgDlE1/iRNBXxdyjhNBwRY4lTm/wRY4lTm/XxdyjgAAAQACAAADAAQABQAABgAHAAgA"
"AAUABgAJAAAKAAsABgAACgAGAAUAAAMABQAMAAAHAAIADQAADgAAAA8AAAAADgABAAAHAAYACwAABQAJABAAABEAEgAIAAAEABMABQAAEAAMAAUAAAcACwAU"
"AAAKAAUAEwAAFAAVAAcABgcAFgACAAYHABEACAAACwAXABQAABgADQACAAAJAAYAGQAACwAKABoABgQAGwATAAYCAAEAGAAAEAAcAAwAAAgAGQAGAAAHAA0A"
"HQAAEAAJAB4ACRkAHwAJAAAHABUAFgAGAQAgABgAABgAIQANAAcRAAcAHQAAEAAeACIACRoAIwALAAYTABsAJAAGBAAlABsABhAAIgAmAAkQACYAHAAACAAn"
"AB8ACQoAEwAoAAAfAB4ACQAAKQAhABgABAgAEgAnAAkAACoADwAGGQAIAB8ACRIAKwAnAAAUACwAFQAGIAApABgABCsAEgARAAAmACIALQAALAAUABcABikA"
"DQAhAAcmAC0AHAAABAADACUABgIAFgAAAAYdAA0ALgAAEQAvACsACRMAJAAoAAYlACQAGwAGHwAwAB4AACoAAAAWAAYuAA0AKQAADAAcADEAAB8AJwArAAAw"
"ACIAHgAAHwArADAAACgAMgAzAAYsABcAKgAGJAAyACgABiUAMgAkAAYaADMAMgAGFgAVACoABhUALAAqAAYPADQADgAGMAAtACIAABEAHQAvAAkgAAEANQAG"
"MQAcADYACTcAOAAaAAYBAA4AOQAAMAArAC8AADIAOgAaAAY7AB0ALgANPAA9AA4AAC0AMAA+AAAgADUAKQAALQA2ABwACT0AOQAOAAA/ADwADgAALgApAEAA"
"AAEAOQBBAAABAEEANQAGKQA1AEIAAC4AQAA7AA0+ADAALwAAQwA2AC0ACUQAPAA/AAAtAD4AQwAARAA/AA4ABjsALwAdAAlBADkARQAAOQA9AEUABhoAOAAj"
"AAY1AEEARgAGQQBFAEYAAEcAQAApAAA6ADcAGgAGRQA9ADwAAEcAKQBCAABFADwARgAAQgA1AEgAAEkAQABHAAAvAEoAPgAAQABJADsADS8AOwBLAAkjAEwA"
"TQAGRgBIADUABjQATQBEAAY0AEQADgAGTgBEAE0ABkoALwBLAAk0ACMATQAGSwA7AEkADUoATwA+AAA4AEwAIwAGPgBQAEMAAEMAUQA2AAlSAFMAVAAGUgBV"
"AFMABlYAVwBYAAZWAFkAVwAGQwBaAFEACU8AUAA+AABKAFsAXAAATwBKAFwAAFgAXQBWAAZUAF4AUgAGXwBdAFgABlQAYABhAAZXAGIATgAGYwBTAFUABlkA"
"YgBXAAZeADcAOgAGVABkAGAABmUAYgBZAAZIAEcAQgAEVgBdAGYABlAAZwBDAABOADwARAAGWABOAF8ABlYAZgBlAAZjAGQAUwAGUABPAGgACWEAXgBUAAZU"
"AFMAZAAGWgBpAFEAAEwAOABqAAZjAFUAawAGXABsAE8AAG0ASQBHAABnAFAAaAAJXABbAGwACU8AbABuAABtAEsASQAAZwBoAEMAAEoASwBbAABaAEMAaAAA"
"bwBsAFsACW4AaABPAAlLAHAAcQAAWQBWAGUABm0ARwBIAAdbAHEAbwAJWwBLAHEAAHIAawBVAAZxAHMAbwAJbgBsAG8AAHMAcABvAAB0AHEAcAAAWABXAE4A"
"BmoAOABmAABLAG0AdQAAbwBwAGkAAG4AbwBpAABjAGsAZAAGaQBaAGgAAGAAdgBhAAB1AHAASwAAcAB1AGUAAHYAZgA4AABiAHUATgAAcgBkAGsAAHMAdABw"
"AAA8AE4AdQAAdQBiAGUAAHUASABGAAB1AEYAPAAASAB1AG0AAGkAaABuAABzAHEAdAANYABmAHYAAGQAZQBgAABmAGAAZQAAcgBpAGQAAHAAZQBpAABlAGQA"
"aQAAdwB4AHkABncAegB4AAZ7AHwAfQAGewB+AHwABn8AgACBAAZ/AIEAggAGgwCEAIUABoMAhgCEAAaHAIgAiQAFhwCJAIoABYsAjACNAAWMAI4AjQAFjwCQ"
"AJEABpEAkACSAAaTAJQAlQAGlgCVAJQABpMAkACWAAaPAJMAlQAGjwCRAJYABo8AlgCUAAaQAJUAlgAGkgCPAJQAAJIAlACTAACRAJMAkgAGkQCTAJYABogA"
"hwCXAACYAI0AjgAAmQCXAJoACZsAmACZAAmZAJoAmwAJhwCcAJoAAJsAmgCcAACaAJcAhwAAigCcAJ0AAIoAnACHAACcAI0AmwAAmACbAI0AAIwAiwCeAACL"
"AI0AnAAAngCcAJ8AAJ4AiwCcAACdAJ8AnAAAoACdAIoABp0AoQCfAAadAKIAoQAAnQCjAKAABqIApAClAAWmAIkApwAApACJAKYAAJ0ApQCoAAClAKMAqAAA"
"owCpAKgAAKkAigCoAAClAKMApAAAowCpAKQAAKAApACpAACKAKQAoAAApACKAIkAAKoAiQCIAACnAIkAqgAApwCrAKYAAKQApgCsAACiAKUArQAAngChAK0A"
"AK0AogCkAACtAKQAoQAAoQCkAJ8AAJ8ApACeAACsAJ4ApAAAngCsAIwAAKwApgCuAACvAI4ArAAArACuAK8AAKwAjgCMAACmAKsArgAAqACtAJ0AAKUAnQCt"
"AACwALEAsgAGsgCzALAABrQAtQC2AAa3ALYAtQAGmACbAJoABrgAuQC6AAC6ALsAvAAAuQC7ALgAALgAugC8AAC5ALsAvAAAvQC+AL8ABr8AwAC9AAaYAJoA"
"lwAG"
;
static const char* CARDATA_WAGON =
"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"
"BQAJIgAjACQABiUACwAKAAYmACcAKAAGCQApAAoABioAGwArAAYaACsAGwAAKQAsAAoAACkADwAsAAAXAC0ALgAAFwAdAC0AAC0AJgAuAAAuAC8AMAAAGgAc"
"ADEAAA8AHwAvAAAVADIAHQAGLgAwABcAABUAFAAyAAYzAB8ABgAAMAAvAB8AACgANAA1AAYKADUANgAGCgAoADUABjcAJAAjAAYoAAoAOAAGCgAsADkABiQA"
"NwA6AAYOADsAOgAALwAsAA8AABMAKwAaAAAuACYAOAAADAAOADwABhMAGgAxAAA8AB4ADAAGCAA9AAYAAD4AFwAwAAA2AD8ACgAGPwBAAAoABgoAQAAlAAZB"
"AEIAGwAGKABDADQABjgAJgAoAAY4AAoAOQAGLAAvADkAADsAJAA6AAYNADsADgAAEwAxABEAABQAHgBEAAA9ADMABgAARQAwAB8AACoAQQAbAAYJAEYAKQAG"
"OAAvAC4AADgAOQAvAAAOADoANwAGDwBHAAcABhMAEgBIAAZJABIABwAGSgBFAB8AAEUASwAwAAAhAEwABQANQwBNADQABk4ADwApAAYZAEoAHwAAMABLAD4A"
"AAUASgAZAA02ADUATwAGGwBCAFAABhMAUQArAAYbAFAAHAAAUQATAEgABkcAUgAHAAZSAEkABwAGHABTABEACTEAHAARAARUAA0AFgAAGAAXAD4AAEwASgAF"
"AA0EAFUAIQANVgAOADcABk4ARwAPAAYSAEkASAAGFABEADIAAB8AMwAZAABXAAAAGQANVQBMACEACQQAAABXAA1XAFUABAANJwBYACgABi0AWQAmAAYtAB0A"
"WQAGVgBaADwAAFoAWwAeAAAdADIAXAAGKgBdAEEABjwADgBWAAY8AFoAHgAAHABeAFMACV8AEQBTAAQRAF8AEAAAYABdACoABkIAYQBQAAYyAEQAYgAGRAAe"
"AFsAAD0ACAAgAAAgAGMAPQAAZAAYAD4AAD4ASwBlAA01ADQATwAGZgAnACYABloAZwBbAAZbAGIARAAGVAAWAGgAAEUASgBMAA1LAEUAaQAJagAWAGQAABgA"
"ZAAWAABVAGsATAAJawBVAFcADSUAbABtAAYrAG4AbwAGIAAQAHAAADMAPQBxAAAzAFcAGQANMwBxAFcAABYAagBoAAByAHMASwAJTwB0ADYABigATQBDAAYl"
"AG0ACwAGdQA3ACMABlYANwB1AAZcAFkAHQAGPgBlAHYACWUASwBzAA0qACsAbwAGdwBTAF4ACXgAdwBeAARbAGcAeQAGaAB6AFQAAHsAZAA+AAlyAEsAaQAJ"
"RwBOAHwABmcAWgBWAAZ9AHoAaAAJRQBMAGsACXsAPgB+AAkrAFEAbgAGXgAcAFAAAH8AUgBHAABiAIAAMgAGagBkAIEADWkARQBrAAlXAIIAawAAgwBpAGsA"
"AH4APgB2AAllAHMAdgANhABCAEEABk0ATwA0AAZmAIUAJwAGKQBGAE4ABmcAVgB1AAZTAHcAhgAEeQBiAFsABhAAXwBwAAB6AIcAVAAAfQCHAHoACWoAfQBo"
"AAlBAF0AhAAGiAB1ACMABiYAWQBmAAZhAF4AUAAARwB8AH8AAEgAiQBRAAZIAIoAiQAGfwCKAFIAAEkAigBIAABJAFIAigAAcgB2AHMADSQAiwAiAAaMAEYA"
"CQAGTgCNAHwABo4AfwB8AACPAH8AjgAGhgB3AHgABGMAcQA9AABqAJAAfQAJZAB7AIEADVEAiQCRAAaNAI4AfAAGfwCSAIoABlwAMgCAAAZwAGMAIAAAcgBp"
"AJMACUIAlABhAAZGAJUATgAGbgBRAJEABmEAlgBeAABZAFwAgAAGYwCXAHEAAHEAlwCYAA1qAIEAkAAJfgCZAHsACYcAmgBUAAB9AJsAhwAAaQCcAJ0AAGkA"
"nQCTAAAlAEAAbAAGYQCeAJ8AAJ8AoABhAABuAJEAoQAGawCCAIMAAJkAfgCiAAB0AKMANgAGpAClAKAABqYAZwB1AAaVAI0ATgAGZwCnAHkABl8AUwCoAABX"
"AHEAggAAmwCaAIcAAHsAmQCiAAA/AGwAQAAGZgCpAIUABnUAiACmAAZZAKkAZgAGpgCnAGcABoAAYgCqAAZ5AKoAYgAGlACeAGEAAKQAoACfAAZhAKAAlgAA"
"qwCnAKYABqcAqgB5AAaYAIIAcQANgwCCAJwACYMAnABpAABdAG0AhAAGiACFAKkABowAbgChAAarAKYAiAAGrACVAK0ABqEArgCvAAatAJUArwAGWQCAAKkA"
"BngAXgCWAARjALAAsQAAbQBsAIQABiQAsgCLAAaMAKEArwAGRgCMAJUABowArwCVAAZwAF8AqAAAcACzALAAAH4AcgCiAAByAH4AdgAAqwCIAKkABoAAqwCp"
"AAarAIAAqgAGqgCnAKsABrEAmACXAA2bAH0AtAAAgQB7AKIAALUAoAClAAa0ALYAmwAAPwCjAGwABoQAtwCUAAakALUApQAGlgCgALUAAKIAuACBAACfALkA"
"pAAGQgCEAJQABrUApAC5AAaGAKgAUwAAYwCxAJcAAJgAugCCAAmbALYAmgAAngCUALcAAJ4AuQCfAAC6AJgAsQANYwBwALAAAJAAtAB9AACQAIEAuAAAcgCT"
"AJ0AAKIAnQC7AACiAHIAnQAAPwA2AKMABrgAtACQAAC8AL0AhAAGvgC5AJ4AALkAvwC1AAa1AMAAlgAAwADBAJYAAKgAswBwAAC7ALgAogAAbADCAIQABlgA"
"iwCyAAa/AMMAtQAGdADEAKMABp4AtwC+AADDAMAAtQAAnADFAJ0AALgAxgC0AACGAHgAxwAAqACGAMcAACgAsgBNAAZ4AJYAwQAAsQCwALoADYIAugCcAAm7"
"AMgAuAAAwwC/AMkABr8AuQDJAAa5AL4AyQAAxwB4AMEAAMkAygDDAAC+AMoAyQAAKABYALIABsEAwADLAAbMALMAqAAAzQDFAJwAAJ0AxQC7AAC7AM4AyAAA"
"hAC9ALcABsEAywDHAAbIAMYAuAAAtwC9AL4AALQAzwC2AADPALQAxgAAnAC6AM0ACbAAswDQAADFAM4AuwAAwwDKANEABscAzACoAADAANIAywAGsADQALoA"
"DboA0ADTAA3EAHQATwAGxgDUAM8AANEA0gDDAAbTAM0AugAJwADDANIABs0A1QDFAAC+ANYA0QAAygC+ANEAAMsA1wDHAADRANYA2AAA1wDMAMcAAMYA2QDU"
"AAC9ANoAvgAA2ADLANIAAMsA2wDXAADcANUAzQAAowDCAGwABtIA0QDYAADYANsAywAATwBNAMQABr4A3QDWAADMAN4AswAA3wDgAM4AAMYAyADZAADOAOAA"
"yAAAzQDTANwACdoA3QC+AADZAOEA1AAA4gDbANgABsgA4ADZAADiANgA1gAG4wDXANsABtQA5ADPAADeANAAswAA1gDlAOIABtsA4gDjAAbQAOYA0wANxQDf"
"AM4AALIA5wBNAAblANYA3QAA6ADlAN0AANUA3wDFAADlAOkA4gAG6QDlAOgAAOkA6ADqAADcANMA6wAJ5wDsAE0ABtUA7QDfAADiAOkA6gAA6ADuAOoABu8A"
"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"
"AYYBAHABkgGTAQCSAXABUQEAXAFwAZMBADABVAFgAQaUARQBJgEGlQFGAVIBBmMBPAGKAQY9AZYBVAEGggFoAWwBBowBjwF8AQCQAVsBlwEAlwFbAX0BAFEB"
"bwGSAQBxARABNQEAiwGYASoBBnIBRgGVAQY9AWEBlgEGZwGWAWEBBmkBmQFmAQaZAYsBZgEGcQGaAXMBBJoBegFzAQSPAZsBaQEAewGPAWkBAEABfQERAQBv"
"AYQBkgEAkAGEAYMBAHQBnAF1AQ11AZwBhQENfwHEAIgBBiYBmAGUAQYmASoBmAEGjQGWAWcBBp0BWgGOAQZ5AY4BWgEGmgFAAXoBAFwBkwFDAQCFAZ4BdgEN"
"nAGfAYUBDaABoQGiAQCBAWMBjAEAZQFyAYIBBoIBeQFoAQaQAaMBhAEAdgGeAYcBDYMBpAGcAQ2fAZ4BhQENfwGIAaUBBhQBiAHsAAYTAV8BiQEGogGlAYgB"
"BmkBmwGZAQaXAaYBkAEAkQFDAacBAKMBkgGEAQB0AYMBnAEN9AB/AaUBBvQApQFdAQYUAZQBoAEGiAGgAaIBBvYAqAFrAQaoAYoBawEGcQE1AYkBAJQBoQGg"
"AQCiAaEBqQEAogGpAaUBAGMBigGqAQaMAWMBqgEGcQGJAZYBAIwBqgGRAQaaAY0BQAEAkwGSAasBAIcBpAGEAQ1eAXgBFAEGiAEUAaABBosBlAGYAQCrAaMB"
"oQEAlAGrAaEBAIsBqwGUAQCjAakBoQEAfgGGAagBAKMBpgGpAQCZAZsBpwEGpwGbAY8BBqMBkAGmAQCrAZIBowEAkwGrAYsBAJ8BrAGeAQmcAa0BnwENdwFT"
"AWsBBl0BpQGAAQZSAYABpQEGiQFfAVQBBlIBcgGVAQBSAaUBqQEAlgGJAVQBAJkBkwGLAQCRAY8BjAEGkwGZAacBAKwBpAGHAQ2HAZ4BrAENUgGpAXIBAJYB"
"jQGaAQCoAaoBigEAmgFxAZYBAKoBhgGRAQCoAYYBqgEAqQGmAXIBAKYBlwFyAQCOAXkBjQEAjQF9AUABAIIBlwF5AQCRAacBjwEAcgGXAYIBAI0BeQF9AQBD"
"AZMBpwEAfQF5AZcBAIMBhAGkAQ2fAa0BrAEJrQGcAaQBCawBrQGkAQmuAa8BsAEGrgGwAbEBBq4BsQGyAQauAbIBswEGrgGzAbQBBq4BtAG1AQauAbUBtgEG"
"rgG2AbcBBq4BtwG4AQauAbkBrwEGugG7AbwBBroBvQG+AQa6Ab8BvQEGugHAAb8BBroBwQHAAQa6AcIBwQEGugHDAcIBBroBxAHDAQa6AcUBxAEGugG8AcUB"
"BsYBxwHIAQbGAcgByQEGxgHJAcoBBsYBygHLAQbGAcwBzQEGxgHNAc4BBsYBzgHPAQbGAc8B0AEGxgHQAdEBBsYB0QHHAQbSAdMB1AEG0gHVAdMBBtIB1gHV"
"AQbSAdcB2AEG0gHZAdcBBtIB2gHZAQbSAdsB2gEG0gHcAdsBBtIB3QHcAQbSAdQB3QEG3gHfAeABBt4B4AHhAQbhAeIB3gEG4wHeAeIBBuMB4gHkAQblAeMB"
"5AEG4wHlAeYBBuMB5gHnAQbnAd8B3gEG5wHeAeMBBugB6QHgAQbgAekB3wEG6QHqAd8BBt8B6gHrAQbsAegB4AEG6wHfAecBBusB5wHmAQbgAe0B4QEG4gHh"
"Ae4BBuIB7gHkAQbuAeUB5AEG7gHvAeUBBvAB7wHuAQbwAe4B8QEG7QHxAeEBBuEB8QHuAQbgAfIB8wEG4AHtAfIBBu0B9AHyAQbtAfUB9AEG4AHzAewBBvEB"
"7QH1AQbxAfUB8AEG9gH3AfgBBvkB+gH7AQb8AfkB/QEG/gH/AQACBv8B/gEBAgD+AQICAQIGAwICAv4BBgQCAgIDAgYEAgUCAgIGBgIHAggCBgkCCAIHAgAJ"
"AgcCCgIGCQIGAggCBgsCBgIJAgYMAg0CDgIGDAIPAg0CBg4CEAIMAgYOAhECEAIGEAIRAhICBhACEgITAgYUAhECDgIAFAIVAhECABUCFgIRAgARAhYCEgIG"
"FQIXAhYCABACGAIMAgYQAhkCGAIGGgIZAhACBhoCEwIQAgANAg4CGwIGHAIOAh0CBg4CFAIeAgYfAiACIQIGHwIiAiACBiICHwIjAgYkAiICIwIGJQIgAiIC"
"ACUCJgIgAgAlAiICJwIAJwIiAiQCBiUCJwIoAgAhAiACJgIAKQIqAh8CBioCIwIfAgYqAisCIwIGLAItAi4CBi0C7AEuAgbsAegBLgIGLwIsAi4CBi4C6AHp"
"AQbpAS8CLgIG6QHqAS8CBuoB6wEvAgYwAi8C6wEGMQIwAusBBi0CMgIzAgYtAjMC7AEG7AEzAvMBBjICNAIzAgYzAvIB8wEGNALyATMCBjQC9AHyAQb1AfQB"
"NAIG9QE0AjUCBjYC9QE1AgY3AjgCOQIAOgI7AjwCADoCPAI9AgA5Aj0CPAIAPgI4AjcCADsCPwJAAgA6Aj8COwIAOQI8AkECADcCOQJBAgA+AjcCQgIAQwJE"
"AkUCAEMCRgJEAgBCAkUCRAIAQgI3AkUCAEYCQwJHAgBIAkYCRwIASQJKAksCAEwCTQJOAgBLAk0CSQIATQJMAkkCAE8CUAJRAgBRAlACUgIGUgJPAlECBlIC"
"TwJQAgBTAlQCVQIAVgJXAlgCAFMCVQJWAgBXAlYCWQIAWgJbAlYCAFYCXAJZAgBdAlUCXgIAVgJVAloCAF0CWgJVAgBeAlUCVAIAXwJBAjcCAGACPAJBAgBh"
"AmICXwIGYgJjAmQCBmUCZAJmAgZnAmgCaQIAagJrAmwCBgsCbQJuAgZvAnACcQIGcgJzAnQCBnICdAJ1AgB1AnQCdgIGagJsAnQCAGwCdgJ0AgZ3AngCeQIG"
"egJ7AnwCBn0CfgJ/AgZ+AoACfwIGfgJ8AoACBn0CgQJ+AgZ/AoICfQIGegKDAoQCBnwCfgJ6AgZ/AoUChgIGfwKAAoUCBocCgwJ6AgaHAnoCfgIGhgJ9AoIC"
"BoACiAKFAgaCAn8ChgIGggKJAooCBosCigKCAgaKAokCjAIGjAKLAooCBogCgAJ8AgaIAnwCewIGgQKGAocCBoECfQKGAgaDAocChgIGjQKOAo8CBnoChAJ7"
"AgaEApACkQIGhAKSAnsCBoQCkQKSAgaQAoQCkwIGlAKRApUCBpQCkgKRAgaDApYChAIGlwKTApYCBpcClQKQAgaUApgCkgIGlQKRApACBoUCmAKUAgaFAogC"
"mAIGmQKQApoCBpACmgKbAgaaApwCmQIGnAKaApsCBnsCkgKYAgaIAnsCmAIGlwKWApUCBpYCgwKVAgadAp4CnwIGoAKhAqICBqMCpAKlAgCmAqQCowIApwKk"
"AqYCAKcCqAKkAgClAqgCqQIGpQKmAqMCBqkCpgKlAgaoAqYCqQIGpwKmAqgCBqkCpwKjAgaqAqsCrAIGrAKqAqsCBq0CMAIxAgYsAq4CLQIGrgIsAq8CBjEC"
"sAKtAgYsAjACrwIGrwIwAq0CBi0CNQIyAgaxAjYCNQIGsgKxAjUCBq4CsgItAgayAjUCLQIGswI2ArECBrQCtQL8AQY2AvoB/AEGNgKzAvoBBrYC8AH1AQbw"
"AbYCtQIGtgL1ATYCBrYC/AG1Agb8AbYCNgIGtwK4ArkCBvcBMQK3Agb3AbACMQIGtwLmAbgCBjEC6wHmAQbmAbcCMQIG9wG3ArkCBroCuwK8AgYJAr0CvgIG"
"vgK9AgcCBr8CwALBAgbCAsMCxAIGxQLDAsICBlkCXAI+Agk4Aj4CXAIJXQI4AlwCCVkCPgJEAglZAkQCVwIJVwJEAkgCCUgCRAJGAglCAkQCPgIJXQI5AjgC"
"CV4COQJdAgleAlQCPQIJVAI/Aj0CCVQCQAI/Agk9AjkCXgIJPQI/AjoCCVwCWwJaAglcAloCxgIJxwLIAgUCCccCBQLAAgkFAgICwAIJyQLKAssCCckCzALK"
"AgnJAssCCwIJ/wG/AsECCc0CGgLOAgnPAs4CGgIJ0AIjAigCCSMC0ALRAgnSAhcCGgIJ0wK6AigCCdQC1QLWAgXXAtgC2QIFvQLLAsoCBQcCvQLKAgXKAswC"
"BwIF/wHHAsACBf8BwAK/AgXaAtsCGAIF2wLPAhgCBc0CzgLbAgXbAs4CzwIFGAIcAtoCBQ8CHAIYAgUbAhwCDwIF0gLNAtsCBRUC0gLbAgXSAhUCFwIFHAIe"
"AtoCBdsC2gIVAgW8AtEC3AIFuwLRArwCBbsCIQLRAgXRAiECKgIF0ALcAtECBdAC3QLcAgW6AtwCKAIFvALcAroCBf0BtAL8AQCzArEC+gEA+gGxAvsBALEC"
"sgLeAgAVAhQC/QEAHQL9ARQCAP0B3wK0AgD9AR0C3wIAFAIeAh0CABUC/QEXAgAXAv0B4AIA+wHgAv0BAN4C4AL7AQCxAt4C+wEA3gKyAuECALICrgLhAgDe"
"AuEC4gIA3gLiAuMCAOACFgIXAgDjAuAC3gIAuQL4AfcBAPYB+AElAgD3Aa0CsAIA9wH2Aa0CAK8C5AKuAgC7AiUC+AEAuQLlAvgBAPgB5QK7AgAlAigC5gIA"
"9gElAuYCAOQC9gHmAgCtAvYB5AIArQLkAq8CAOQC4QKuAgDkAuIC4QIA5gIoAicCAOYC4gLkAgD/AecC6AIA6QLqAscCAMcC6ALpAgDrAukC7AIA6wLtAukC"
"AOsC7gLtAgDqAu0C7wIA6QLtAuoCAP8BAQLnAgDpAucC8AIA5wLxAvACAPIC8ALzAgAHAvIC9AIA6QLyAuwCAOwC8gL1AgDsAvUC9gIA9gLrAuwCAO8C9QLq"
"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"
"rE32hRH6Dk77hfL6BE1ehhn6A07UhZ35903OhVT5YE0qhoj6/kw7hke2Fiqrgl0UbEDyhoLtX03FiBEVD02qiEvtKUDMhqb4VHMvmXm5HnD7naJJLnNOnktP"
"JFy5jNzU1k2zioS7Gkc0iuC1lU+tivqcIEfdka881TXThwzG6Tf/hVxDnT76hzr5a0phic87WjTphSnJ1zP+hiIA9E84hi9Mb1gHje6zPFsZjfQIKl3khuJH"
"y2+qnQAAAQACAAADAAQABQAABgACAAcAAAYABwAEAAAIAAcAAgAACQAKAAsABgwADQAGAAYDAAoACQAAAAAOAA8AABAABAAHAAAMABEADQAGDQACAAYAAAUA"
"CgADAAAAAA8AAQAABAADABIAABEAEwANAAYUAAAAAgAGFQAWAAcAABcAAgANAAYHABYAEAAADgAYABkABgAAGgAOAAYOABkADwAAGgAAABQABhcAFAACAAYV"
"AAcAGwANHAADAAkABggAGwAHAA0IABUAGwAJBgAdAAwABgYABAAeAAYSAAMAHAAAAQAfAAIAACAABAAQAAAWABUAIQAJBAAgAAUAABwACQALAAYSAB4ABAAG"
"GAAOACIABiMAAQAPAAkkACAAEAAJJQAWACEACSQAEAAmAAkOABoAIgAGIQAVAAgACScAIQAIAAAmABAAJQAJEAAWACUADQ0AEwAXAAYoABwACwAGKAASABwA"
"BiAAKQAFAAkGAB4AKgAGHwAIAAIAACsAHwABAAAYACwAGQAGGQAtAA8AACAAJAApAAkpAC4ABQAAIQAnAC8AAAgAHwAnAAAwADEADAAGHgASACgABhgAEQAs"
"AAYTACIAGgAGIwAPAC0ABAoAMgALAAYTABoAFwAGKAALACoABh4AKAAqAAYtABkAMwAAJgAuACQAADMAGQA0AAYjACsAAQAAHwArADUAACkAJAAuAAAlACEA"
"LwAAJgAlAC8AABEADAAxAAYvAC4AJgAAEQAxACwABh0ACwAyAAYZACwANAAAKwAjAC0AAAwAHQAyAAYtADMANgAGNwAvACcAACcAHwA3AAkzADQAOAAGMwA4"
"ADYABh8ANQA5AA05ADcAHwAJNwA6AC8AADYAKwAtAAAuADsAPAAALAA9ADQAADQAPQA4AAAuAC8AOwAAPgA2ADgABi8APwA7AAA2AD4AQAAGNQBBADkADTIA"
"QgAMAAY+ADgAPQAAOgA/AC8AADcAOQBDAAlEAEAAPgAAKwA2AEUAAEUANgBAAABBAEMAOQANCgBGADIABkMAOgA3AAk1ACsAQQAAPwBHADsAADEAPQAsAAZI"
"ADoAQwAJPQBJAD4ABkAARABFAAQ/ADoASAAAPAA7AEoAAD8ASwBHAABMACsARQAAMABNADEABk4APQAxAAY9AE8ASQAGMAAMAEIABk4AUABRAAZOAFIAUAAG"
"SQBEAD4AAFMAVABVAAYrAEwAQQAAMQBSAE4ABlQAVgBVAAZQAFIAVwAGUwBYAFkABjsARwBaAABFAFsATAAAOwBaAEoAAFQAXABWAAZDAF0ASAAAPQBOAE8A"
"BkoAXgA8AABIAF8APwAJVQAyAEYABkcAYABaAAkxAE0AUgAGYQBiAGMABksAPwBfAAlkAGIAUgAGVwBSAGIABmUAWABTAAZZAGYAXAAGXgBKAGAACUoAWgBg"
"AAlnAEgAXQAJRwBLAGgADV8AaABLAA1iAGkAYwAGRQBEAFsACWYAagBcAAZIAGcAXwAJTgBRAE8ABmMATABhAAZHAGgAYAAJWQBYAGYABkMAQQBMAABrAGcA"
"XQAJZwBrAGwADVMAVQBtAAZlAGYAWAAAXQBDAGwAAGAAZQBeAABoAGwAYAAAbABoAF8AAG4AQgBWAAZpAGIAZAAGZQBTAG0ABlEAbwBPAAZsAEMATAAAXwBn"
"AGwAAGoAbgBcAAZhAG8AUQAGVgBcAG4ABkwAbwBhAAZjAGkATAAGTABpAGwAAGUAYABqAABNADAAZAAGMABuAGQABkYAbQBVAAZEAEkATwAARABPAG8AAGAA"
"bABqAAAwAEIAbgAGagBsAGQAAG4AagBkAABsAGkAZAAAagBmAGUABk0AZABSAAZbAEQAbwAAbQBeAGUAAEYAXgBtAABvAEwAWwAAXQBsAGsADXAAcQByAAZw"
"AHIAcwAGdAB1AHYABnQAdwB1AAZ4AHkAegAGeAB7AHkABnwAfQB+AAZ8AH8AfQAGgACBAIIABoMAggCBAAaDAIQAhQAGgwCFAIIABoYAhwCCAAaHAIIAhQAG"
"gQCEAIMABoEAiACEAAaAAIgAgQAGggCAAIYABoAAiQCGAAaKAIsAjAAGjQCOAI8ABo8AjgCQAAaOAJEAkAAGkgCNAJMABo0AjgCUAAaVAJYAlwAGmACWAJUA"
"BpkAmgCbAACbAJoAnAAAnQCeAJ8ABp0AnwCgAAadAKEAogAGoQCdAKAABp0AogCeAAajAJ4AogAGoACeAKMABqAAnwCeAAaiAKEAowAGpAClAKYAAKcApQCk"
"AACoAKUApwAAqACpAKUAAKYAqQCqAAamAKcApAAGqgCnAKYABqkApwCqAAaoAKcAqQAGqgCoAKQABocAqwCGAAasAK0ArgAGrwCtAKwABrAAmQCbAAmwAJsA"
"sQAJsQCbAJwACbAAmgCZAAmwALIAmgAJsgCzAJoACbQAtQC2AAmRAJAAjwAJjACKAIsABY8AkgCRAAWSAJQAkQAFtwCVAJgABbcAlwCVAAWTAIgAgAAAgACJ"
"AJMAAIkAhgC4AACRAI4AkwAAjgCUAJMAALgAkQCTAACJALgAkwAAhgCrALgAAIQAlgCFAACFAKsAhwAAhQCWAKsAAJYAhAC3AACWAJgAuQAAqwCWALkAAKsA"
"uQC4AAC6ALsAigAAjACLALoAALUAvAC9AAC6ALUAuwAAuwC1ALQAALsAtACKAAC8ALUAugAAvgCKALYAAIoAvgC/AAC2AMAAvgAAvgDAAMEAAMEAvwC+AACT"
"AI8AiAAAkwCSAI8AAI8AkADCAACyALAAsQAAlwC3AIQAAA=="
;
static const char* CARDATA_CROSS =
"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"
"ADwAPQAACAAJAD4ACTcABgAEAAAJABUAPgAJHwAWACwAADEAPwAyAAYuADQAQAAAKwAqAEEAADAAMgBCAAZDACsAQQAAHQAwAEIAACwALwAfAAAZACsAQwAA"
"GQBDABoAAB0AQgBEAABFAEYAIwAACgAdAEQAABsAHwAZAAA2ADUAMwAARwALAAoAACIAEgA2AAAmACcAEQAANwAgAEgAAAsASQAOAAAUAEoADwAAFAATAD0A"
"ADwAEwA6AAAuAEsATAAAHQBNADAAAE4AGgBDAAAlAE8ADQAADwBQACMAAAsAUQBJAAlKAFIADwAAOAAOAFMACR4AOwBUAAAIAD4AOwAJOwA3AAgACVUABgA3"
"AAAVAAYAVgAAVwBYAEUABjEAMABNAAZZAEMAQQAAKwBOAFkAAEMAWQBOAABOACsAGQAAWgAbABoAAFAAWwAjAAAgAFoASAAARwBcAFEAAAsARwBRAABPACUA"
"XQAADgBJAFMAAFIAUAAPAABeAEoAFAAAOQAkACEAADcABAAIAAk8ADoAXwANFQBWAD4AACwAFgBgAAAsAGEAKgAAPwBiADIABjIAYwBkAAZCADIAZAAGZQAr"
"AFkAAC8AKwBlAABmAEAANAAGQgBnAEQAAC8AZQBOAABOABkALwAAHQBoAE0ABiMAWwBFAABaACAAGwAAIwBpABAABjgAXQAlAABdADgAagAANwBrAFUAACgA"
"HgBUAABsAFQAOwAJPQBtABQAAFUAVgAGAABuAEsALgAAYQBvACoAAHAAYgA/AAZYAHEAcgAAMQBNAHMABkUAdABGAAZCAGQAZwAATQBoAHMABiMARgB1AAZP"
"AF0AdgAAJwAmAHcABl0AagB2AABrAEgAWgAASQBRAHgACUgAawA3AAApACgAeQAAOgAkAHoAACkAewA5AABfAD0APAANegB8ADoAABYALgBMAAB9AFcARQAG"
"fgBZAEEABmUAfwCAAAZOAIAAgQAGCgBEAFwABnUAaQAjAAYKAFwARwAGWwCCAEUAAGkAgwAQAAZaAIQAhQAASQB4AFMAAIYAOABTAABrAIcAVQAASgCIAFIA"
"AD4AbAA7AAk5AHoAJAAAewApAHkADV8AOgB8AA1WAGwAPgAJYgBjADIABn0AWABXAAZFAIkAfQAGWAB0AEUABkUAggCJAABOAIEAGgAGUACKAIsAAIQAjABq"
"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"
"AUIBCTIBOAEtAQYrAUsBPgENIgFMASwBAAsBOwFNAQkbAU4B9AAGTwFDASgBAD8BUAEiAQkaAVEBIwEAUQFSASMBAAsBTQFBAQAyAUYBOAEGSwErAfAAADkB"
"UwFUAQBMASIBUAEAGgE0AVEBACABRAEuAQZVAUQBJQEAMQEkAUUBBhABVgFPAQBAAVUBJQEAPQFIAUoBBj4BVwE7AQkjAVIBPwEJPAFHAVgBABUBOQFAAQAo"
"ARABTwEALwFZATcBBi4BWgEvAQZAAVsBVQEAXAFTATkBAFoBWQEvAQYhATMBSgEALgFEAVoBBlgBXQE8AQY8AV4BSAEGQAFUAVsBACEBXwEOAQBAATkBVAEA"
"LAFgATkBADUBGAFCAQkbAfQA0QAGLAFMAWABAD8BYQFQAQliAU0BOwEJUQFjAVIBAEUBVgEQAQBEAWQBWgEGXQFYAUcBADwBXQFlAQZIAWYBSgEA8ABnAWgB"
"APAAaAFLAQBBAWkBQgEJagFCAWkBCWsBQwFPAQBcAVQBUwEFSwFoAT4BDTsBVwFsAQlHAW0BXQEAZgEhAUoBAPAADgFfAQBuAU0BYgEJUgFjAT8BBgUBbwEk"
"AQZrATIBQwEGbQFHAUYBBl4BPAFlAQZIAV4BcAEAXwFnAfAAAGcBcQFoAQA1AUIBcgEJXgFlAXABBmYBcwEhAQBUAVwBWwEFOQFgAVwBAHQBYwFRAQB0AVEB"
"NAEAcgE6ATUBCSQBdQF2AQZFASQBdgEGdgFWAUUBAGYBSAFwAQBcAXcBWwEFRgEyAW0BBnABZQF4AQBgAXkBXAEAPwF6AWEBBmMBegE/AQZXAT4BaAENbwF1"
"ASQBBnsBbQEyAQZEAVUBZAEAeQF8AVwBBU0BbgF9AQlNAWkBQQEATgF+AfQABk8BfwFrAQBrAXsBMgEGcwFfASEBAGEBegGAAQljAYEBegEGdAE0AYIBADQB"
"OgGCAQB7AWsBfwEAbQF7AYMBAF0BbQGDAQBdAYMBZQEAhAFkAVUBAGUBgwF4AQBmAXABeAEAcQGFAVcBCXEBVwFoAQBNAYYBaQEAcgGHAToBAPQA1QCYAAZb"
"AYQBVQEAXwGIAWcBAIABYAFMAQBiATsBbAEJNwFZARsBBk8BVgF/AQZ/AYkBewEAgAFMAVABAGkBigFqAQBZAYsBTgEGeAGMAWYBCVwBfAF3AQVQAWEBgAEJ"
"egGNAYABCY4BegGBAQZMAJgA1gAGiQF/AVYBBoMBjwGQAQaDAZABeAEAdwGEAVsBAFcBhQFsAQljAZEBgQEAggGSAXQBAIcBggE6AQCKAUIBagEAgwF7AY8B"
"AJEBjgGBAQBjAXQBkQEAkwGUAXIBAEIBkwFyAQBZAU4BGwEGewGVAY8BAIkBlQF7AQCWAYwBkAEAeAGQAYwBAGcBiAFxAQCAAY0BYAEAggGXAZIBAGkBhgGK"
"AQCYAXUBbwEGmQF2AXUBBloBmgFZAQaQAY8BmwEGZgGMAXMBCXMBnAFfAQBsAYUBnQEAjgGeAXoBBn0BhgFNAQCXAYIBhwEAkwFCAYoBAIcBcgGUAQCKAZ8B"
"kwEAVgF2AZkBBmQBmgFaAQaIAaABcQEAjQF5AWABAHoBngGhAQCeAY0BoQEGjQF6AaEBCZEBogGOAQCiAZ4BjgEAkQGjAaQBAKMBkgGlAQmmAZEBkgEAowGm"
"AZIBCaMBkQGmAQCnAZIBlwEAbgGoAX0BAJIBkQF0AQCnAZcBqQENhwGqAakBDakBlwGHAQ2HAZQBqgEAmQGJAVYBBo8BlQGrAQaPAasBmwEGjAGWAXMBCawB"
"iAFfAQCeAaIBjQEArQGuAYoBAJ8BigGuAQ2GAa0BigEAdQGvAZkBBpoBsAFZAQaNAbEBeQEAogGxAY0BAKQBogGRAQCkAaMBpQEJqgGUAakBCYYBfQGoAQCU"
"AZMBsgEJrQGGAagBAKgBswGtAQmtAbQBnwENmgFkAbUBAHwBtgF3AQWcAawBXwEAtwF8AXkBBbQBrQGzAQ2uAa0BnwENdQGYAa8BBk4BuAF+AQZkAYQBtQEA"
"tgGEAXcBAHMBuQGcAQCxAbcBeQEGkgGnAaUBCWIBnQG6AQC0AZMBuwEJnwG7AZMBALQBuwGfAQCVAYkBvAEAkAGbAZYBALYBvQGEAQC+AaIBpAEJpQG+AaQB"
"CW4BugGoAQBuAWIBugEAiwG/AU4BBrABiwFZAQbAAawBnAEAfAG3AbYBBaUBpwGpAQmpAcEBwgEJqQHDAaUBCcQBrwGYAQZvAcQBmAEGxQG/AYsBBsUBiwGw"
"AQa1AbABmgEAnQFiAWwBAKgBugHGAQmUAbIBxwEJqQHCAcMBDcEBqQGUAQmTAcgByQEJkwG0AcgBCbMByAG0AQ3KAcQBbwEGTADWAMsBBmAATADLAQYFAWAA"
"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"
"AgZDAkcCSAIGQwJIAkkCBkMCSQJKAgZDAkoCSwIGQwJLAkwCBkMCTAJNAgZDAk0CTgIGQwJOAkQCBk8CUAJRAgZPAlICUAIGTwJTAlQCBk8CVQJTAgZPAlYC"
"VQIGTwJXAlYCBk8CWAJXAgZPAlkCWAIGTwJaAlkCBk8CUQJaAgZbAlwCXQIGWwJdAl4CBlsCXgJfAgZbAmACYQIGWwJhAmICBlsCYgJjAgZbAmMCZAIGWwJk"
"AmUCBlsCZQJmAgZbAmYCXAIGZwJoAmkCBmcCagJoAgZnAmsCagIGZwJsAmsCBmcCbQJsAgZnAm4CbQIGZwJvAm4CBmcCcAJvAgZnAnECcgIGZwJpAnECBnMC"
"dAJ1AgZzAnUCdgIGdgJ1AncCBnYCdwJ4AgZ5AnoCewIGeQJ8AnsCBn0CfgJ/AgZ/An4CgAIGfwKAAoECBoECgAKCAgaBAoICgwIGgwKCAoQCBoUChgKHAgaF"
"AocCiAIGiAKHAokCBooCiwKMAgaMAosCjQIGjAKNAo4CBo8CkAKRAgaPApICkQIGkgKRApMCBpIClAKTAgaVApYClwIGlQKWApgCBpgCmQKWAgaYApkCmgIG"
"mgKbApkCBpoCmwKcAgacAp0CmwIGnQKeAp8CBp8CoAKeAgagAqECngIGogKjAqQCBqICpQKjAgalAqYCpwIGpgKoAqcCBqYCqAKpAgapAqoCqAIGqQKqAqsC"
"BqsCrAKqAgarAq0CrAIGqwKuAq0CBq4CrwKwAgauArECrwIGsQKyAq8CBrECswKyAgazArQCsgIGswK0ArUCBrUCtgK0Aga1ArcCtgIGtwK4ArYCBrcCuAK5"
"Aga5AroCuAIGuQKiAroCBrsCvAK9Aga7Ar4CvQIGvgK/Ar0CBr4CwAK/AgbBAsICwwIGwQLDAsQCBsQCwwLFAgbEAsUCxgIGxwLIAskCBsgCygLJAgbLAsoC"
"zAIGzQLOAs8CBs4C0ALPAgbOAtEC0AIG0gLTAtQCBtIC1QLTAgbVAtMC1gIG1wLYAtkCBtcC2ALaAgbaAtgC2wIG2gLcAtsCBtwC2wLdAgbeAt8C4AIG3gLg"
"AuECBuIC4wLkAgbjAuUC5AIG4wLmAuUCBuYC5wLlAgbmAugC5wIG5gLoAukCBukC6ALqAgbpAusC6gIG6wLsAuoCBusC7QLsAgbtAu4C7AIG7QLvAu4CBu0C"
"8ALvAgbwAvEC7wIG8ALyAvECBvIC8wLxAgbyAuMC4gIG8gLmAuMCBvIC6QLmAgbyAusC6QIG8gLtAusCBvIC8ALtAgb0AvUC9gIG9gL1AvcCBvYC9wL4Agb4"
"AvcC+QIG+AL5AvoCBvoC+QL7Agb6AvsC/AIG+gL8Av0CBvoC/gL9Agb+Av8C/QIG/gL/AgADBgAD/wIBAwYAAwEDAgMGAAMCAwMDBgADAwMEAwYEAwMDBQMG"
"BAMFAwYDBgQD9AL2AgYEA/YC+AIGBAP4AvoCBgQD+gL+AgYEA/4CAAMGBwMIAwkDBgcDCgMJAwYKAwsDCQMGCgMMAwsDBgwDDQMLAwYMAw4DDQMGDAMPAw4D"
"BgwDEAMPAwYQAxEDDwMGEAMSAxEDBhADEgMTAwYTAxIDFAMGEwMUAxUDBhMDFgMVAwYWAxcDFQMGFgMYAxcDBhgDCgMHAwYYAwwDCgMGGAMQAwwDBhgDEwMQ"
"AwYYAxYDEwMGGQMaAxsDBhkDGwMcAwYcAxsDHQMGHAMdAx4DBhwDHgMfAwYfAx4DIAMGHwMgAyEDBiEDIAMiAwYhAyIDIwMGIQMjAyQDBiEDJAMlAwYlAyQD"
"JgMGJQMmAycDBiUDKAMnAwYoAykDJwMGKAMpAyoDBioDKQMrAwYZAxwDHwMGGQMfAyEDBhkDIQMlAwYZAyUDKAMGGQMoAyoDBiwDLQMuAwsvAzADMQMKLwMx"
"AzIDCjMDNAM1AwozAzUDNgMKNwM4AzkDCjcDOQM6Awo7AzwDPQMKOwM+AzwDCv+g9zNNcymVRkRMZ9+U1CHTZ9SZwjreTzWdTUQhWeCaP1T7mV+PKTmyqnmW"
"ozx9uIWQLDSnxp6ZOFhFwmaeclrDuuKThhpXNqCa1R3qOjOXpTGMPqCXSVWH+amPRBAPJQeUNxedz7SUHxrBnnycYA4czDSRk1EbImKcPxEsZt6T5xCRcaWU"
"WA2VNmSX1Q4HPbieQhl6c+WWpxtKNsSddkn6jvyaUg9XPFypKykah0mZvVRc7mKhNwyZBe+cvAzKPUfBswsQZX2iRjB8p8ycm139ICqktA9ij2efAg5ppAms"
"p3BL61bBQQwaPkrF0A4MhVy/gxYgfVfGZDf6e5vECkGzen68iVnjiH2zNGGllZSomGbeps+vjXEkxLSsEXFECq3CzlamO8q5JFj/MlHGmicbfF+6vnS/Ehiv"
"wXfP9LTJdAu0eGuwxnS7vhO5aHGvm13BPwy0y1PBQwy2pB7cbnboln9AQwzDy1heBgyz98TJfntGB6EAu0PGf6bj602ohZLqhVqYPRLLgH6cxvHHjV7ylbD2"
"klj5N4dAQwy3pEnqFiuAfoNAQwwbPgwaHSoBgI38GHbgFhT4F3uGCt82+nu4ByXqVR01fjRedw4gi5NAxwscZf4GskavhUYVGR+tfqVCoBbAfCI0J3gfmlUY"
"tykefuc1x34Bx9050WH7l0s2ygvfeOc6IUK1eehSml9ul7FE7HTuEt5Rr1mpi7c1SEyIh9Ujd3N3mBY5Djjre1Jcs0blbjhIHVZUN287yicNfJVloQ5PpOFQ"
"xXOfxL9Rc3FWCq5PeXDRq4VT03C07ItcEBpUdORmezgIbqVeh0pITABTwnOq9Q1sag+ocRpbZSZVibxi9l7gHyZfg1yfq8JoHCoPc1hls1gcwTBjnkm6kDZq"
"D0JtX7hgxlr4u31lK1Tr+eFnSFQ37pRlsxUpZlZmfgxsPQJrJE+sKJFpARjINblgtSZSOy5kaFQ2m61rNlKUwKVmjg0BzCRwdiESLU1itxfhnyJvryfBn4Zp"
"PDpJS9tzclYMIchsIxPYOJ51JDXGYKRuNhxbOF9x8ECRtBRioTu2tsxrGBi8N/BvmB0lOktwLSahzSdupA/IXyduMi9YSSduzyqpXCdu2gV/TSdudxVkQleT"
"pA/IX1eTzyqpXFeTMi9YSVeTdxVkQleT2gV/TSdukwtXwyduhjBMvCdu6CBoxyduLgcGsCduvSYCqleTvSYCqleT6CBox1eThjBMvFeTkwtXw1eTLgcGsAlH"
"MBvIZw9HpjJOYnxrNxvwZ2pzxDc3XbWRkDgoWZdpWDqAVQtHWzgFR3dh6inGPA9HNBteOmls5h3ZO3S6MBvIZx6VNhvxZ2669jruXA2MkjXDXmGcYjqEUTBn"
"+jmzTcmc8TjlSWeefiscPjePDSMSPEu3CxskO/pGTxuzzx2SMBt2zgJtuyAtzxhH8TyhuhtxjDohsCqiQzYfrcpcEDDCpquiIjPzqQJHORt/oAxiOxvEoWu6"
"Shsa0U+Urhxr0XW6Dj0VsRGRzjmmst6nuzHPqidfSC82p2OiBivGpKGmMhsSogAAAQACAAADAAIABAAABQAGAAcAAAcACAAJAAAHAAkACgAACwAMAA0AAA4A"
"DwANAAAOAAgAEAAAAQAEAAIAAAUAEQAGAAAFAAcACgAAEAASAA8AAA4AEAAPAAAOAA0AEwAAAgAUABUAAAsAFgAXAAALABcADAAAAgAVABgAAAAAAgAYAAAN"
"AA8ACwAAGQAMABcAAAUAGgARAAAWABkAGwAAFwAbABkAABkAFgALAAAaABwAEQAAHQAIAA4AABYADwAeAAAWAB8AFwAAFAAgABUABgwAFgAbAAAPABYADAAA"
"DwAMABkAABkACwAPAAARACEABgAGIgAOABMACQ8AEgAeAAAcACMAEQAAEwAEACIAAB0ACQAIAAARACMAJAAGEwANAAQAAAQADQADAAAdACUACQAJHwAWAB4A"
"AB4AJgAfAAYjACcAJAAGFQAoABgAABEAJAAhAAYHAAYAIQAGAAApACoACQUAKwAaAAUqAAEAAAAJBQAKACwAAAoALQAsAAAdAA4AJQANIwAcACcABgoACQAu"
"AAkJACUALgAJDgAiAC8ACSIAMAAxAAApAAAAMgAAKwAFACwAADMALwAiAAAlAA4ANAANIAA1ABUABjYACgAuAAkOAC8ANAAJLwAzADQAACwALQA3AAAtAAoA"
"NgAJOAAeABIABiQAJwA5AAYtADYAOgAAOAA7ADwABgAAGAAyAAYzAD0ANAAAMAAiAAQAACIAMQAzAAAuACUANAAAAQAqAD4AAD8AHAAaAAA3AC0AOgAAKAAV"
"ADUABgEAQAAEAAAwAAQAQAAANgAuAEEAADQAQQAuAAAYACgAMgAAKwA/ABoABSwAQgArAAAxAEMAMwAJOQBEADgABjoANgBBAABCACwANwAGRQApADIAAD4A"
"QAABAAA9AEEANAAAMABDADEADR4ARgAmAAYpAEUAKgAGRwAnABwABjMAQwBIAAk6AEIANwAGSQBKAD0ACSgASwBFAAYoAEUAMgAGJwBMADkABkkAPQAzAABG"
"AE0AJgAGSQAzAEgAACAATQA1AAY/AE4AHAAATQAgACYABk4ARwAcAAAwAEAAQwANSgBBAD0ACU8ASwAoAAYoAFAATwAGKgBFAD4ABkEAUQA6AABSAEUASwAE"
"UQBBAFMAAEcATAAnAAZFAFIAPgAGOgBUAEIACVIASwBPAAQrAE4APwAASABKAEkAAEEASgBTAAlNAFUANQAGPgBSAFYABkIAVwArAABIAEMAWAAJVABXAEIA"
"CUgAWABKAAArAFkAWgAATgBaAEcAADoAWwBUAAkoADUAUAAAUgBcAFYABj4AXQBAAABOACsAWgAATABEADkABisAVwBZAAA6AFEAWwAAVQBQADUAAEAAXQBe"
"AABAAF4AQwANQwBeAFgACVIAXwBcAABbAFEAVAAGXwBSAE8ABlYAXQA+AABPAFAAXwAGTABgAEQABlEAUwBhAABYAGIASgAARAA7ADgABlcAVABRAAYeADgA"
"PAAGYQBjAFEAAGQAYQBTAABKAGQAUwAAUABlAF8AAGYAXABfAABdAGcAXgAASgBiAGgAAGkAVQBNAAZHAGoATAAGVgBcAF0ACVgAXgBrAABkAEoAaAAAUABV"
"AGkABlEAbABXAABfAGUAbQAGXABmAF0ACWwAWQBXAABjAGwAUQAAYQBkAG4ACVkAbwBaAAVwAGcAXQAARwBaAGoAAGMAYQBsAAllAFAAaQAAXwBtAGYAAFgA"
"awBiAABhAHEAbAAJYgBrAHIACWEAbgBxAA1oAHMAZAANdABpAE0ABkYAHgA8AAZGADwAdQAGXgBnAHYAAHcAeAB1AAZZAGwAeQAAbgB6AHEADTsAewA8AAZr"
"AF4AdgAAZABzAG4ACWgAYgByAAlsAHEAegAAcgBzAGgADXsAfAA8AAB9AGAATAAGbwBqAFoAAFkAeQBvAAVMAGoAfgAAeABnAH8AAHgAdgBnAABrAIAAcgAJ"
"dQA8AHwAAHwAgQB1AAB9AEwAfgAAdwB8AHgABmcAggB/AACDAHgAfAAGfgBqAG8AAH0AhACFAAaCAGcAcAAAbwB5AH4AAGwAhAB5AAB1AIYAdwAAdQCBAIMA"
"AHUAgwCGAAB8AIYAgwAAcABdAGYACXIAfABzAAByAIAAdgAJbAB6AIQAAHYAgABrAACBAHcAhgAAhgCHAIEAAIcAgwCBAACHAIYAfAAAfgCEAH0ABnwAcgB2"
"AABuAIgAegAAgQB8AHcAAGUAaQB0AABtAGUAdAAAeACDAIcABm0AdACCAAB8AHYAhwAAhwB2AHgAAIgAfAB7AACIAHMAfAAAZgBtAIIAAG4AcwCIAACCAHAA"
"ZgAAfgB5AIQAAIQAegCIAACJAIoAiwAGiQCMAI0ABokAjQCKAAaOAI8AkAAGjgCRAJIABo4AkACRAAaTAJQAlQAGkwCWAJcABpMAlwCUAAaYAJkAmgAGmACb"
"AJkABpgAnACbAAadAJ4AnwAGngCgAJ8ABp4AoQCgAAaeAKEAogAGngCjAKIABqMApACiAAajAKUApAAGpQCmAKQABqUAngCdAAalAKMAngAGpwCoAKkABqkA"
"qACqAAapAKoAqwAGqQCrAKwABqkArQCsAAapAK0ArgAGrgCvALAABrAApwCpAAawAKkArgAGsQCyALMABrEAtACzAAa0ALUAswAGtAC2ALUABrQAtgC3AAa3"
"ALYAuAAGuQC6ALcABrkAtACxAAa5ALcAtAAGuwC8AL0ABr0AvAC+AAa9AL4AvwAGvQC/AMAABr0AwQDAAAa9AMEAwgAGuwC9AMIABg=="
;
static const char* CARDATA_BIGSUV =
"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"
"AAABAAIAAAMABAAFAAAAAAIABgAABgACAAcAAAgACQAKAAALAAwADQAACgADAAgAAAQAAwAKAAADAA4ACAAABgAMAAsAAAsAAAAGAAAPAAAACwAAAwAQAA4A"
"ABEAAwABAAAQAAMAEgAAEwAUABUABhUAFAAWAAYIABcACQAAEwABAAMAABUAAQATAAAVAAIAAQAAAgAYAAcAAAkAGQAKAAADAAUAGgAADAAbABwAAA0ADAAc"
"AAAGAAcAHQAAHQAMAAYAAB4AAQAAAAASAAMAEQAAHwAQABIAABcAIAAhAAYTACIAFAAGFwAIACAAABoAIgATAAAVACMAAgAAGAACACMAABMAAwAaAAAkABwA"
"JQAAJAANABwAAAsADQAmAAQnAA8ACwAAHgAoAAEAABEAAQAoAAApAAAADwABFAAqABYABisAIAAIAAAOACsACAAALAAKABkABi0AGwAMAAYnAAsAJgAAKQAu"
"AAAAAC8AEgARAAAQAB8AMAAALgApADEAARUAFgAyAAYiADMAFAAGMgAjABUABi0AHAAbAAYmAA0ANAAEHgAAAC4AADUAKAAeAAA2ACgANQAADwA3ACkAAR8A"
"EgA4AAAyABYAKgAAIAA5ACEAABcAIQA5AAYnADoAOwAANQAeAC4AATgAEgAvAAEfADgAMAABMAA4AC8AATcAMQApAAA8ADEANwAAFAA9ACoABisAOQAgAAA+"
"ACUAHAAAPwAcAC0ABh0AQAAMAAEnACYAOgAAOgAmADQAACcANwAPAAEvABEAKAABPAAuADEAABQAMwA9AAY9AEEAKgAGOQArAEIABkMAIwAyAAZEACwAGQAG"
"BwBFAB0AAUYARwAKAAFHAEgACgABCgBIAAQAAScASQA3AAFJACcAOwAASgA1ADwAATwANQAuAAFKADYANQABSwAwAC8AAUsAEAAwAAAzAEEAPQAAJQA+AEwA"
"Bk0AGAAjAAZIABoABQAGTQAHABgABi0ADABOAAZIAAUABAABPAA3AEkAASgATwAvAAE2AE8AKAABUAAqAEEABjMAUQBBAABSADMAUwAGIgBTADMABlQACQAX"
"AAZVAD4AHAAGPwAtAE4ABEsAVgAQAABXAEsALwABVwAvAFgAAVIAUQAzAABUABkACQAGWQA7ADoAAEoAPABJAAFYAC8ATwABUwBRAFIABlEAUABBAABUABcA"
"OQAGHAA/AFUABlUAPwBaAAY/AE4AWgAERABGACwABkUAQAAdAAFbAEkAOwAAVgBLAFcAAFwAXQBXAAFDADIAKgAGPgBVAF4ABhoAUwAiAAZEABkAVAAGCgAs"
"AEYAAUkAWwBfAABJAF8ASgABXABXAFgAAVAAQwAqAAZNACMAQwAGGgBgAFMABk0AYQAHAAYMAEAAYgAGYQBFAAcAAU8ANgBKAAFcAFgAYwABXQBWAFcAAEgA"
"YAAaAAZiAE4ADAAGXwBbAGQAAFYAXQBcAABlAFAAUQAGZgBZADoAAFwAYwBWAABjAFgATwAAQgBUADkABmcAOgA0AABoAGQAWwABaQBfAGQAAGkASgBfAABq"
"AE8ASgAAUwBlAFEABmsATAA+AAZeAGsAPgAGRQBiAEAAAWkAZABsAABKAGkAagAAVgBjAG0AAGMAbgBtAABqAGMATwAAWgBOAFUABjsAbwBbAAFsAGQAaAAA"
"agBuAGMAAGAASABwAAZhAE0AQwAGRQBhAGIABmIAYQBOAAZHAEYASAAGOwBZAG8AAXEAXgBDAAZTAGAAcAAGVQBOAF4ABkMAXgBOAAZEAFQARgAGYQBDAE4A"
"BkYAcABIAAZGAFQAcAAGcABUAHIABmsAXgBxAAZnAGYAOgAAWwBvAHMAAXQAUwBwAAZyAHQAcAAGWQB1AG8AAXUAWQBmAAFzAGgAWwABUwB2AGUABmkAbABq"
"AAB3AEMAUAAGeABUAEIABmwAaABzAAB0AHkAUwAGcwB6AGwAAG4AewBtAABUAHgAcgAGfABqAGwAAEMAdwBxAAZyAH0AdAAGcQB+AGsABn0AeQB0AAZ8AH8A"
"agAAgAB1AGYAAYEAfQByAAaCAIAAZgAAbwB6AHMAAYIAZgBnAACCAHUAgAAAbwCDAHoAAYMAbAB6AAB+AIQAawAGagB/AG4AAIUAbwB1AAFuAIYAewAAhwBs"
"AIMAAIIAhQB1AACDAG8AhwABiAB3AFAABngAgQByAAZ/AIYAbgAAiQCCAGcAAIUAigBvAAF5AHYAUwAGdwCLAHEABosAfgBxAAZlAIwAUAAGggCNAIUAAIUA"
"jQCKAAF4AI4AgQAGdgCMAGUABocAbwCPAAGQAI4AeAAGjwBsAIcAAJEAfwB8AACMAIgAUAAGkgCCAIkAAIIAigCNAACPAG8AkwABjwB8AGwAAH0AgQB5AAaG"
"AJQAewAAigCVAG8AAZUAkwBvAAGWAIQAfgAGlwB5AIEABn4AiwB3AAaYAH4AdwAGgQCZAJcABncAmgCYAAaYAJoAmwAGmwCcAJgABoEAnQCZAAadAJ4AmQAG"
"kACfAI4ABnkAlwCgAAadAKEAogAGnQCiAJ4ABoIAkgCKAAB5AKAAdgAGowCaAHcABp4ApACZAAabAKUApgAGpQCbAJoABpoApwClAAaoAJIAiQAAqQCVAIoA"
"AYEAjgCdAAabAKoAnAAGqwCsAKcAAacArAClAAGTAK0AjwABrgB8AI8AAJQAhgCvAACvAIYAfwAArwB/AJEAALAAlgB+AAalALEApgAGlwCyAKAABqoAmwCm"
"AASuAI8ArQAArgCzAHwAAHwAtACRAAB8ALMAtAAAkQC1AK8AALAAfgCYAAapAIoAkgAAtAC1AJEAAK8AtgC3AAB3AIgAuAAGoACMAHYABp8AuQCOAAa6AK4A"
"rQAAlACvALcAALYArwC1AAC5ALsAjgAGmAC8ALAABpwAvACYAAa9AJ0AjgAGnACqAL4ABr8AmgCjAAaqAKYAwAAEnQDBAKEABsIAugCtAAC3AMMAlAAAtgDE"
"ALcAAbgAxQB3AAagAMYAjAAGxQCjAHcABqcAmgC/AAaiAKEAwQABvwCrAKcAAccAngCiAAGTAMgArQAAwgCtAMgAALoAwgCuAAG0ALMAyQABtgC1AMoAAcsA"
"zACZAAasALEApQABqwDNAKwAAcIAyADOAAHOAK4AwgABzgCzAK4AAckAswDOAAHGAM8AjAAAmQDQAJcABr4AvACcAAamALEArAAGzgDIAJMAAMoAxAC2AAGg"
"ANEAxgAAxgDRAM8AAM8AiACMAADQALIAlwAG0gCjAMUABtMAvwCjAAa+AKoAwAAGzgCTAJUAAMMAtwDEAADRAKAAsgAG0gDTAKMABpkAzADQAAbUAKQAngAB"
"1QCSAKgAAMkA1gC0AAHKALUAtAABvQCOANcABqQAywCZAAapAJIA2AAA2ACVAKkAAMkAzgDZAAHaAMQAygAB2wDEANoAAcMAxADbAAC7ANcAjgAGwQCdAL0A"
"BsEAxwCiAAHHANQAngAByQDZANwAAdgAzgCVAADdAM4A2AAB2QDOAN0AAcoAtADeAAG4AIgAzwAGsADfAJYABqwAwACmAAbgAN0A2AAB4QDdAOAAAeIA3gC0"
"AAHDANsA2gAA0QC4AM8ABuMA0QCyAAa7ALkA5AAA5QDjALIABtAA5QCyAAa8AN8AsAAAnwDkALkAANQAywCkAAHVAKgA5gAEkgDVAOAAAOAA2ACSAADnANYA"
"yQAA5wDoANYAAOcA4gDoAAHoALQA1gAB6QDaAMoAAOoAygDeAADqAN4A4gAB4gC0AOgAAesA3ADZAADdAOEA2QAA0QDjALgABuwAxQC4AAbSAMUA7AAGnwDt"
"AO4AAM0AwACsAAbVAOYA7wAE1QDvAOAAAMkA3ADrAADnAOoA4gAAygDqAOcAAMMA2gDwAADZAOEA6wAA7AC4AOMABuQA1wC7AAbkAJ8A7gAA3wC8AL4AAKsA"
"8QDNAAHrAOEA4AAA2gDpAPAAAOkAygDnAADjAOUA8gAG4wDSAOwABuMA8gDSAAbkAL0A1wAAzADLANAAANIA8gDzAADSAPMA9AAAvgD1AN8AAPIA5QDpAADp"
"APMA8gAAwADNAPYAAL4A5gD1AAD2AO8A5gAE7wD2AOAAAPMA5wDrAADJAOsA5wAA9ADTANIAAO4AvQDkAADQAOkA5QAA0ADLAOkAAO4AwQC9AADAAPYAvgAA"
"7gDHAMEAAPYA8QCrAAD2AM0A8QAAvgD2AOYAAPMA6wD0AADtAPAA7gAA9ADgAPYAAO4A8ADpAAD0AOsA4AAA6QDnAPMAAL8A0wD0AADLANQA6QAAvwD0AKsA"
"ANQAxwDpAACrAPQA9gAAxwDuAOkAAPcA+AD5AAH6APkA9wAB+wD5APoAAPcA/AD4AAX4AP0A/AAF/gD/AAABAQEB/wD+AAH9AAEB/gAF/AD9AP4ABQIBAwEE"
"AQYFAQYBBwEGCAEFAQcBBgkBBwH7AAADAQoBCwEGAwEMAQoBBg0BDgEKAQAKAQ4BDwEAEAERARIBBRIBEQETAQURARMBFAEBEwEUARUBARUBDwEUAQD7AAkB"
"+gAAFgEXARgBARYBGAEZAQEZARgBGgEBDgEPARUBABkBGwEaAQEbARABGQEFGQEQARIBBRwBHQEeAQEdAR8BHgEBIAEhASIBBSMBJAElAQUmAScBKAEBKQEq"
"ASsBASwBJgEoAQErASoBLQEBLAEoAS4BASoBLwEwAQExASoBMAEBMgEzATQBATMBNQE0AQE2ATcBOAEBHgE5AR8BATkBOgEfAQE5ATsBOgEBOwE8AToBCTsB"
"PQE8AQY9AT4BPAEJPQE/AT4BBj8BQAE+AQZBAUIBQwEFRAFFAUYBBUYBRQFHAQlFAUgBRwEGRwFIAUIBBUgBSQFCAQU8AUoBSwEFTAFGAUcBBkwBRwFCAQVN"
"AU4BRgEJTgFEAUYBBUUBRAFPAQVMAU0BRgEJQQFCATsBBVABQQE7AQVLAVEBUgEFUwFUAVUBBVYBVwFYAQlYAVcBWQEGWgEfAVsBAVoBWwFcAQFcAVsBXQEB"
"HgFeAV8BAV8BXgFcAQFeAVoBXAEBXwFcAWABAVwBXQFgAQFhAWIBYwEJYwFiAWQBBmUBZgFnAQVoAWUBZwEFaQFqAWgBBWkBZwFqAQVnAWYBagEFawFsAW0B"
"BWsBbQFuAQVuAW8BcAEFbwFrAXABBWoBZgFxAQVvAXIBawEFawFyAWwBBXMBdAF1AQB2AXcBeAEFdgF5AXcBBXoBewF8AQV8AXsBfQEFeAF3AX4BBX8BgAGB"
"AQaCAYABfwEGgAGDAYQBBoABggGEAQaCAYUBhAEGggGGAYUBBoQBhwGFAQaAAYMBgQEAgQGIAYMBAIUBhgGJAQaFAYkBhwEAhwGKAYsBAIcBiwGJAQCHAYwB"
"igEAiAGNAYMBAI0BjgGDAQCPAZABkQEBkgGTAZQBAZUBlgGXAQaYAZUBlwEGmQGVAZgBBpoBlQGZAQaXAYEBfwEAlwGIAYEBAIkBmQGGAQCLAZkBiQEAkQGQ"
"AZYBBpoBkwGSAQabAZEBlgEGmgGSAZwBBo8BkQGbAQGcAZIBlAEBnQGPAZsBAZcBngGIAQCcAZ8BlAEBoAGZAYsBAIgBngGOAQCMAaABiwEAmAGXAZ4BBpkB"
"mAGgAQaNAZgBngEGjgGNAZgBBqABmAGKAQaMAZgBigEGgwGOAYQBBoQBjAGHAQaEAZgBjgEGjAGYAYQBBqEBogGjAQWkAaUBpgEApwGkAaYBBqQBqAGpAQan"
"AagBpAEGqgGrAawBAK0BrgGvAQCwAbEBsgEAsQGzAbIBBrIBswG0AQazAbUBtAEAswG2AbUBALcBuAG5AQC6AbsBvAEAvQG+AboBALoBvAG+AQC/AbsBugEG"
"wAG7Ab8BBrsBwQHAAQbAAcEBwgEGvgHDAbwBAMQBwAG/AQbFAcABxAEAxgHFAcQBAMYBxAHHAQC+AcMBvQEAxgHFAcgBAMcByAHGAQDJAcoBywEAzAHJAcsB"
"AM0BzgHPAQDPAc4B0AEA0QHSAdMBANEB1AHSAQDVAdEB0wEA1gHUAdEBAD0B0gHUAQDWAdEB1wEA2AHVAdMBANcB0QHVAQDZAdUB2AEA2QHaAdUBANcB1QHa"
"AQDZAdgB2wEA3AHbAdgBANwBPgHbAQA+Ad0B2wEA3gGuAd8BAN4B4AGuAQDeAd8B4QEA3gHhAeIBAOEB3wHjAQCuAcMB5AEA5QHkAcMBAK4BvQHDAQCvAeQB"
"5QEA4AHmAa4BAOUBwwG8AQDlAbwBwQEA5gG9Aa4BAOcBrwHlAQDnAeMBrwEA5wHhAeMBALoBvQHoAQDlAcEBwgEA6QHlAcIBAOgBvQHmAQDpAcIBxQEA6QHF"
"AcgBAOkByAGqAQDHAaoByAEArAHpAaoBAOoBugHoAQDEAb8BugEAugHqAcQBAOsB6QGsAQDsAasBqgEA7QGqAccBAO4BqgHtAQDtAccB7wEA7gHsAaoBAOsB"
"rAHwAQDwAawBqwEA7gHxAewBAPEB8AHsAQDHAcQB6gEA8AGrAewBAMcB6gHvAQDyAfMB9AEA9AHzAfUBAF4B8gH0AQDyAR4BOQEA9gFeAfQBAF4B9wHyAQD0"
"AfUB+AEA9wEeAfIBAPYBHQFeAQD2AfQB+AEA9gH4AfkBAB0B9gH6AQAcAfcBXgEAHQEcAV4BAB8BHQH6AQD7AfwB/QEA/gH/AfwBAP8B/QH8AQD/AfsB/QEA"
"AAL7Af8BAAEC/wECAgACAv8B/gEAAQIAAv8BAAACAQICAgADAgQCBQIABgIHAgQCAAYCBAIDAgAEAggCBQIABQIIAgkCAAoCCwIMAgANAgwCDgIADQIKAgwC"
"AAwCCwIPAgALAhACDwIAEQISAhMCBhQCEwISAgYVAhYCFwIAFwIWAhgCAAEB/wAZAgH9ABkCAQEB+AAaAhsCAfgAGwL9AAH5APgAGgIBHAL5ABoCBvsA+QAc"
"AgYEAQsBHQIGBwEcAvsABgYBHQIcAgYGARwCBwEGBAEdAgYBBgoBDwEdAgYYARoBHgIBEAEfAhEBAREBHwIgAgERASACFAEBFAEdAiACBhQBDwEdAgYWASEC"
"IgIBCwEKAR0CBiMChgGCAQaWASQCJQIAlgElApcBAJcBJQKCAQaZASYCJwIAmQEnApoBACYCmQEjAgaXAX8BggEGhgGZASMCBpABJAKWAQaaAScCkwEGKAIp"
"AioCBioCKQIrAgalASwCpgEGLQKnAS4CBi4CpgEsAgYuAqcBpgEGLwIwAjECBzECMAIyAgcwAi8CMwIHMQIyAjQCADUCMwI2AgAvAjYCMwIANQI3AjMCADUC"
"NAI3AgA3AjQCMgIANwIyAjACADcCMAIzAgA4AjkCOgIGOAI7AjkCBjgCPAI7AgY8Aj0COwIGPgI/AjgCBjoCPgI4AgY/AjwCOAIGOwI5AjwCBjkCPwI8AgY/"
"AjkCPgIGOgI5Aj4CBkACQQJCAgdDAkECQAIHRAJFAkYCAEMCQAJHAgBBAkQCQgIARAJBAkUCAEMCRwJGAgBFAkMCRgIARQJIAkMCAEUCQQJIAgBIAkECQwIA"
"SQJKAksCBkwCSgJJAgZKAk0CSwIGSwJNAk4CBk4CTQJPAgZLAkwCSQIGSwJOAlACBksCUAJMAgZOAlACSgIGTgJKAk0CBlACTAJKAgZRAlICUwIGUQJUAlIC"
"BlUCVgJXAgZVAlgCVgIGWQJaAlsCClwCXQJeAgpfAmACYQIGYQJiAmACBmECYwJiAgZjAmQCYgIGZAJlAmYCBmYCZwJoAgZnAmkCaAIGZwJqAmkCBmcCawJq"
"AgZsAmsCbQIGbAJtAm4CBm4CbQJvAgZuAm8CcAIGcAJvAnECBnICcwJ0AgZyAnMCdQIGdQJ2AnMCBnUCdwJ2AgZ3AngCdgIGdwJ5AngCBnkCegJ4AgZ5AnsC"
"egIGewJ8AnoCBnsCfQJ8AgZ9An4CfwIGfgKAAoECBoACgQKCAgaAAoICgwIGgwJyAoICBoQChQKGAgaEAocChQIGhwKIAoUCBocCiAKJAgaJAooCiAIGiQKK"
"AosCBosCigKMAgaLAowCjQIGjQKMAo4CBo0CjwKOAgaPApACkQIGjwKSApACBpICkwKQAgaUApUClgIGlQKWApcCBpUCmAKXAgaZApoCmwIGmwKEApoCBpwC"
"nQKeAgafAqACnAIGoQKeAqICBp8CngKhAgafApwCngIGowKfAqECBqQCpQKmAgakAqYCpwIGqAKkAqcCBqYCqQKnAgamAqkCpQIGqgKrAqwCBq0CrAKuAgas"
"AqsCrgIGrwKwAqoCBq8CqgKsAgavAqwCrQIGoQKvAq0CBrECsgKzAgaxArQCsgIGtQK2ArQCBrUCtAKxAga1ArECswIGtwK4ArkCBLoCuwK8Aga9AroCvgIE"
"uwK6Ar0CBLkCuAK/AgS5AsACvwIGtwK5ArwCBsECtwK8AgbBArwCuwIGuQK8AsACBrwCwALCAga6AsICvAIGwwK+AsICBsICugK+AgbEAsUCxgIExwLFAsQC"
"BMcCyALFAgDJAsoCywIAxQLLAskCAMgCzALNAgbGAsUCzgIEzgLFAskCBMsCxQLIAgDLAsgCzQIGzwLQAs0CBs8C0QLQAgbSAtMCzAIG0ALMAtMCBs0C0ALM"
"AgbUAtUC1gIE1wLVAtQCBNgC2QLaAgDZAtoC2wIA1gLcAtsCBNsC3ALdAgTeAtcC1AIA2gLfAt4CBt8C1wLeAgbaAt4C2wIA3gLUAtsCANQC1gLbAgTfAuAC"
"2gIG4QLgAt8CBt8C4gLXAgbjAuQC5QIG5gLjAucCBuMC6ALnAgbjAuUC6AIG6QLqAusCBuoC7ALrAgbsAu0C6wIG6wLtAu4CBu4C7QLvAgbtAvAC7wIG7QLx"
"AvACBvEC7QLsAgbyAuwC6gIG6QLzAuoCBvQC9QL2Agb1AvQC9wIG+AL5AvcCBvoC+QL4Agb6AvsC+QIG/AL9AvsCBvwC/gL9Agb/AgADAQMGAgMDAwQDBgUD"
"AgMGAwYCAwcDAwMGBQMIAwIDBgcDAgMIAwYJAwoDCwMGDAMNAw4DBg8DDQMMAwYQAxEDEgMGCgMQAxIDBhADEwMUAwYQAw8DDAMGEAMMAxMDBhUDFgMXAwYT"
"AxgDFQMGFQMYAxYDBhkDGgMbAwYcAx0DHgMGGwMaAxcDBh8DCQMgAwYfAyEDCQMGCQMeAyADBiIDHgMjAwYgAx4DJAMGIwMlAyIDBiMDJgMlAwYXAycDGwMG"
"FwMoAycDBigDFwMWAwYoAxYDJwMGHgMdAyMDBikDKgMWAwYJAwsDHAMGCwMrAxwDBgkDHAMhAwYsAwkDLQMGGAMPAxYDBhcDDAMOAwYTAxUDDgMGDAMXAxMD"
"BhMDFwMaAwYuAy8DMAMGMAMxAy4DBjIDMAMxAwYyAzADFwMGMwMXAzQDBjUDNgM0AwY3AzgDNQMGOQM3AyUDBiUDOgMiAwYlAyIDOwMGJQM8AzsDBjwDOwMe"
"AwY8Ax4DPQMGPgM9Az8DBj4DPwNAAwYlAzcDOgMGNAMXAzADBjUDNAMwAwY6AzcDHgMGMAMvA0EDBjADFgMyAwYfAyADLAMGGwMnA0IDBjADQQNDAwZEA0MD"
"RQMGPQNEA0UDBkYDPQNHAwZFA0cDPQMGQwNIA0UDBh4DIQMcAwZDA0EDSAMGMwMyAygDBjMDKAMXAwYoAzIDFwMGMwM0AycDBiIDJQMkAwY/Az0DSQMGNgMz"
"AzQDBiADPQNKAwZLA0wDTQMGTgNNA0wDBk8DUANRAwRRA1ADUgMEUwNPA1QDBFMDUANPAwRVA1ADUwMEVgNVA1cDBFADVQNWAwRYA1UDUwMEWQNaA1sDBFsD"
"WgNcAwRdA1wDWgMEXQNaA14DBF8DXANgAwRgA1wDYQMEYQNcA10DBGEDXQNiAwRjA2QDZQMGZgNkA2MDBmYDZwNkAwZlA2gDYwMGaQNqA2gDBmgDagNjAwZm"
"A2sDZwMGZgNsA2sDBmsDbQNuAwZtA28DbgMGcANxA2cDBnADcgNxAwZpA3MDagMGcwNrA2wDBmoDcwNsAwZwA2cDdAMGcAN0A3UDBnADdQN2AwZ1A3QDbgMG"
"dANrA24DBnADdwNyAwZ2A3UDeAMGeAN1A3kDBnkDdQNuAwZ6A3ADdgMGdgN4A3sDBngDeQN7AwZ6A3YDfAMGeQNuA30DBnsDfAN2AwZ7A34DfAMGewN5A30D"
"Bn0DfgN7AwZ3A3ADfwMAgAN6A3ADBnoDfAOAAwZ8A4EDggMGfAN+A4EDBn0DgwN+AwZ9A4MDbgMGhANwA4ADBoMDbgOFAwaEA4YDcAMGhwN8A4IDBoEDfgOI"
"AwZvA4kDbgMGfAOHA4YDBnwDhgOAAwaKA4gDfgMGgwOKA34DBosDbgOJAwaMA24DiwMAhgOEA4ADBoMDhQOKAwaNA44DjwMGjgOQA48DBqYCkQOSAwaTA6YC"
"kgMGoQKUA54CBp0CngKwAgaUA6ECngIG5QK0ArYCBugC5QK2AgaVA7MC6AIG5wKVA7MCBpYDlwOYAwaWA5kDlwMGmgObA5wDBp0DngOfAwagA50DnwMGoAOf"
"A6EDBqIDowOkAwajA6UDpgMGpwOoA6kDBqoDqwOsAwarA6oDrQMGrgOrA60DBq4DqQOrAwarA6kDrAMGIgM6Ax4DBq8DOAM3AwYeAzsDJAMGJAM8AyADBkkD"
"PQNGAwY9AyADPAMGJgOwAyUDBksDsQOyAwBLA7MDsQMATAOzA0sDAEwDtAOzAwC0A7UDswMAswO1A7YDAEwDtwO4AwC5A7QDTAMAuAO5A0wDAEwDugO3AwC6"
"A0wDSwMAuQO7A7QDALwDSwO6AwC9A0sDvAMAvgNOA0sDAL0DSwNOAwC0A78DtQMEwAO9A04DAMEDvgOyAwC+A0sDsgMAwgNOA74DAMMDTgPCAwDEA8MDwgME"
"wgO+A8UDAMIDxgPEAwTHA8QDxgMExQPGA8IDAMgDyQPKAwDKA8kDywMAyAPMA8kDAM0DzgPJAwD+As0DyQMAzwPIA9ADBMwD0QPJAwDPA8wDyAMEyQPSA9MD"
"AMwD1APRAwDJA9UD0gMAyQPTA9YDAMkD1gP+AgDTA/sC1gMA+wLTAz0DANQDzAPPAwDRA9UDyQMAzgPNA9cDAPwC1gP7AgDXA80D2AMA2QPUA88DANED2gPV"
"AwDbA80D/gIA1APcA9EDANQD3QPcAwDbA94DzQMA/AL+AtYDANgDzQPeAwDfA9QD2QMA0QPgA9oDAOED1APfAwDhA90D1AMA3QPiA9wDAOkC2wP+AgD9AvMC"
"/gIA6QL+AvMCANwD4APRAwDhA+ID3QMA4gPgA9wDAPoC+wI9AwDiA+MD4AMA4QPkA+IDAOUD4QPfAwDkA+YD4gMA5gPjA+IDAOcD5APhAwD9AuoC8wIA+gI9"
"A0QDAPgCRAP6AgDhA+UD5wMA+wLqAv0CAOgD5gPkAwDnA+kD5AMAQwNEA/gCAOoD6QPnAwDpA+sD5AMA6gPnA+UDBOwD6wPpAwD3AvgCQwMAMAP3AkMDAPkC"
"6gL7AgD5AvIC6gIA6QPqA+0DAOkD9gLuAwDsAvIC+QIA7gP3AjADAPEC7AL5AgD3AvEC+QIA7wPpA+0DAPYC6QPvAwDuA/YC9AIA9wLuA/QCAPcC9QLxAgDv"
"A+0D8AMA9gLvA/ADAPYC8AL1AgD1AvAC8QIA9gLvAvACAPYC8APxAwD2AvED7wIA7QPyA/ADAPMD9AP1Awb2A/UD9AMGvAP3A7oDBvgDugP3Awb5A7wDvQMG"
"+QO9A/oDBr4DvQP6AwC+A/oD+wMAugP8A/0DBroD/QO3AwbAA/4D+wMGwAP7A74DBrcDswP/AwC3A/8D/QMAAAS4A7MDBgAEswP/AwYBBMADTgMGAQROA8MD"
"BrkDAgS7AwbDA8QDAwQGuwMEBLQDBMcDBQQGBATHAwYExAMEtAMHBL8DBAgEtQO/AwR/A3ADCQQACQRwAwoEBoYDCgRwAwZuAwsEhQMGCwRuAwwEBgwEbgOM"
"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"
"AGcAbQBmAABtAG4AZgAAbwBnAGUAAGsAcABqAABxAHIAbwAGcABzAHQABm8AdQBnAAB2AHAAawAAZwB1AG4AAGgAdgBrAABwAG8AdgAGbQBvAHUABm4AbQBv"
"AAZ2AG8AbAAGaABvAGwABmYAbgBoAAZoAG8AbgAGdwB4AHkABnkAeAB6AAZ7AFcAfAAAewB8AH0AAHsAfQBYAAB+AH8AgAAAfgCAAIEAAIAAggCDAAB/AIIA"
"gAAAgwCCAIQAAIMAgQCAAACDAIQAhQAAhgCDAIUAAIYAhQCHAACGAIcAiAAAhwCEAIIAAIQAhwCFAACJAIgAhwAAigCIAIkAAIsAhgCIAACMAGIATgAATgCN"
"AI4AAIwATwBiAACMAE4AjgAATwBOAGIAAFEATwCMAACPAJAAkQAAkgCTAJQAAJMAlQCUAACWAJcAmAAAmQCaAJsABpwAmwCaAAZFAEMAnQAGngCfAJ0ABp4A"
"nQBFAAagAEwAnwAGSQBIAEsAAUsATACfAAaeAKAAnwAGbwChAGUABqIAcABqAAZyAKEAbwAGcACiAHMABqMApAClAAajAKYApAAGpwCoAKkABqcAqgCoAAar"
"AKwArQAGrgCvALAABrAArwCxAAayALMAtAAGtQCzALIABrUAsgC2AAa3ALMAsgAGtwC2ALMABrIAswC4AAa5ALoAuwAGtgC3ALQABrYAsgC4AAa8AL0AsgAG"
"uQC6ALsABrgAsgC9AAa9ALMAvAAGuwCvAL0ABr0AvgC7AAa4ALwAsgAGvwDAAMEABsIAwwDEAATFAMMAwgAExgDHAMgABMgAxwDJAATKAMsAzAAGzQDOAMwA"
"BswAzgDKAAbKAM8AywAGygDOAM8ABs0AzwDOAAbQAMsAzwAG0ADRAMsABtAAzwDSAAbQANIA0wAG0wDUANAABtUA0ADUAAbTANIA1gAGzwDXANIABtgA2QDa"
"AAbZANsA2gAG3ADdAN4ABt0A3wDgAAa7ALoAuQAGwADhAOIAAMAA4wDhAADBAOMAwAAAwQDkAOMAAOQA4QDjAADBAMAA5QAA5gC/AMAAAOcAwAC/AADoAOcA"
"vwAA5gDAAOIAAL8A5gDiAADpAOoA6wAA7ADrAOoAAOkA7QDqAADqALsA7AAAsADsALsAAO4A7wDtAAC7AK4AsAAA8ADxAO4AAK8ArgC7AADxAPAA8gAA8wDx"
"APIAAPEA8wCxAAC9APEAsQAAvQCxAK8AAPQA9QD2AAb3APYA9QAG5wD4AMAABvkAwAD4AAbmAOcA+gAA5gD6APsAAMAA/ADlAAboAPsA5gAG5QDjAP0AAOUA"
"/QD8AAD9AMEA4wAG4gD+AL8ABOQA/wDhAATVANEA0AAG0gDXANYABgABAQECAQYDAQEBAgEAAAEEAQEBBgQBBQEAAQDfAN0A3AAG"
;
static const char* CARDATA_MINIVAN =
"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"
"sas4cTUpya9IcvUlnOjTdfQmqMN2dHYlbupddZInvD/udNYkvz+0dIolK1GDcogmRVvhV1JS1FavbTAw4FmkVXVSoMzFfUvMVNgYf7TIKNgTf/zHOiNOf9DH"
"wiH/f33IQjS4fQrMtzeUfTjinzakfgTPGTZjfdDkPCqffsDnWSqYfnro49W+fo/oFdbGftbnyMnDfdDiPtCYe9sIse1afREGTst0fGIKu+1kfcoGYix3fCwH"
"Iy4KffAH+SoGeckaqgRhegIbpgSZek0a1tU8eUUcadGqeIQbFV/PZtcwmFohcaIeJF6IZxYxuFtycH4eRFldcmQKMGL5Yds5819iYTo7+F9aVE9KeWPcWEme"
"yWWvV76+OWQAWEaeYWWeWMC+AAABAAIABwMAAQAAAAADAAAABAAAAQADAAIABwUABAAAAAAAAAIABgAHBwAGAAIABwMABwACAA0DAAgABwANBAAIAAMADQUA"
"AAAJAAAKAAsADAAACwAKAA0AAA4ADwAQAAARAAoADAAADAALABIAABMAFAAVAAAWABcAEwAAFgATABgAABMAFQAZAAAMABIAGgAAGwAcAB0ABx0AHAAeAAcf"
"ACAAIQAHIgAfACEABwAABgAjAAckACUAFQAAJQAmABUAAAoAJgAnAAAoACkAKgAAKwAsACQABi0ALgApAAAVACYALwAAKwAkABUAACgALQApAAAPABYAEAAA"
"FAArABUAABEAJgAKAAApAC4AMAAAEgALADEAABYAGAAQAAAyADMAMAAANAA1ADAAABkAFQAvAAAzADQAMAAAIQAgAC8AABoANgA3AAAYABMAOAAAOQAjAAwA"
"ABoANwA5AAAvACAAGQAALwAeABwAAC8AEQAeAAA2ADUANwAAOAA6ABgAAAwAGgA5AAARAAwAOwAAHgARADwAACEALwAcAAAcABsAIQAAOwA9ABEAAB4APAAd"
"AAA3AD4AOQAAIgAhABsAAB8AGQAgAAAZAB8AIgAAOwAjAAYAABMAGQA4AAAJAAAAIwAHJwANAAoAAD8AQAAqAAYlACcAJgAAQQALAA0ABjAAQgApAAAyADAA"
"LgAAEwBDABQAAEMAEwAXAAAmABEALwAARAAzADIABDQAMwBEAAQ2AEUANQAARgA1ADQAAEcANQBGAAA1AEcASAAAIwA7AAwAAEgAPgA3AABJABgAOgAAPABK"
"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"
"AQBgAWEBMgEHYQFZATIBB1QBRwE7AQdiAV4BWwEHXQFcAWMBB14BZAFcAQdLARoBVgEEQAFlAUoBBmYBYgFbAQf5AGcBKgEAVQFfATkBB1ABNAEsAQEwASsB"
"LQEAZABoAT8BB2kBagFmAQdrATgBKgEAPgFsAScBAEoBbQHmAAdOAW4BbwEGbgFOAT0BB3ABZwH5AABnAWsBKgEAVQFxAXIBB3IBXwFVAQcvAXMBPAEAGwF0"
"AS0BAEYBQwEzAQZtATcB5gAHdQE0AVABASkBMAFEAQBgAXYBYQEHcQF3AXIBBjgBawFWAQA0ATUBFQEAPAF0ARsBADMBeAFGAQZlAXkBbQEGTgFvAVoBBkEB"
"egE9AQd7AXwBfQEGagFpAX4BBn8BTwE6AQeAAUsBVgEESwGAAVgBAHYBWQFhAQaBAWgBZAAHPQF6AW4BBzQBggE1AQAvAS4BcwEJLQGDAUQBAEQBhAEpAQDC"
"AIEBZAAHZwFwAYUBB2oBfgGGAQaHAYgBiQEGVwFIAYoBB4IBNAF1AQAtAXQBiwEAgQE/AWgBBkoBZQFtAQZmAYwBaQEGfAGNAXcBBjYBSQFNAQ08AXMBdAEA"
"PgGEAWwBAIQBPgEpAQA/AYEBjgEGRgGPAZABAPkAkQFwAQBrAZIBVgEAkwGKAUgBB3UBUAGUAQSVAWwBlgEBlgFsAZcBAZYBlwGEAQGWAYQBmAEBmQGDAS0B"
"AC0BiwGZAQCEAZcBbAEAdgGaAVkBBsIARgGQAQCQAZsBwgAAnAGdAZ4BBnABnwGFAQCFAWsBZwEHfAF7AaABBncBjQGhAQaGAaIBowEGcgGhAaQBB5IBgAFW"
"AQSUAVABWAEEggF1AZQBBKUBLgGmAQClAXMBLgEApwGoAakBCXQBcwGqAQCrAawBrQEJpwGrAZkBAYsBrgGnAQGqAa8BdAEAmAGwAbEBCawBsgGZAQGsAZkB"
"rQEBrQGZAasBAacBmQGLAQGLAXQBrwEAsQGzAZgBCbABmAG0AQGwAbQBrAEBrAG0AYMBAawBgwGyAQGYAYQBtAEBsgGDAZkBAIQBRAGDAQC0AYQBgwEARgF4"
"AY8BBmUBtQF5AQZ7AX0BtgEGtwFaAW8BBnIBdwGhAQa4AV8BcgEHkwFIAV8BB0kBpgFNAQBNAaYBLgEArwGqAbkBCboBqgG7AQm7AaoBcwEAqwGnAakBCaUB"
"uwFzAQC8AbABrAEBjwF4Ab0BBlkBmgFRAQZ2Ab4BmgEAYAG+AXYBAIwBfgFpAQaNAXwBoAEGiAGjAYkBBn4BogGGAQakAaEBjQEAVwF/AToBB6QBuAFyAQdJ"
"ATUBggEAugG7AaUBAasBqQG/AQmrAbwBrAEJwAGuAa8BAagBpwHBAQGxAbABvAEBrgGLAa8BAW0BkQE3AQB6AW8BbgEHkQGfAXABAIgBhgGjAQaAAcIBWAEA"
"ggHDAUkBAMMBpgFJAQDDAcQBpgEAugGlAcUBAcUBpQGmAQmmAcYBxQEJugHHAaoBCasByAG8AQm5AcABrwEJyQGuAcABAbMBlQGWAQmzAZYBmAEJwQGnAa4B"
"AT8BjgG2AQZtAXkBkQEA+QA3AZEBAGYBWwGMAQaHAcoBnAEGygGdAZwBBp8BawGFAQdPAXoBQQEHywGSAWsBAMYBpgHMAQm5AaoBzQEJxwHNAaoBCaYBxAHM"
"AQmzAc4BlQEJqwHPAcgBCasBvwHQAQnRAbMBsQEJ0gGpAagBAagBwQHJAQCuAckBwQEA0wFRAZoBBtQBeQG1AQbVAZABjwEAmwGBAcIAAMsBawGfAQeJAcoB"
"hwEGoAHWAY0BBo0B1gGkAQa4AZMBXwEHkgHCAYABBMIBlAFYAQDEAdcB2AEJ1wHEAcMBANkB2gHHAQHEAdgBzAEJxwHbAc0BAbkB3AHdAQHcAbkBzQEBugHe"
"AccBCd4B2QHHAQnAAbkB3QEByAHfAeABAcgBzwHfAQHhAeIBzwEB4gHQAb8BAeABvAHIAQnhAc8BqwEJ0AHhAasBCb8BqQHSAQHRAbEB3wEB3wG8AeABAagB"
"yQHSAQCxAbwB3wEB4wGSAcsBAOQBggGUAQTYAeUB5gEA5wHMAdgBAdkB6AHaAQDcAdsB6QEA2wHHAdoBAdwBzQHbAQHdAckBwAEB6gHOAbMBAbMB0QHfAQDi"
"Ad8BzwEB4gHhAdABAesB4gG/AQHrAb8B0gEByQHrAdIBALoB7AHeAQHCAeQBlAEA5AHDAYIBBOUB1wHDAQDlAdgB1wEA7QHMAecBANoB6QHbAQDpAe4B3AEA"
"yQHdAe4BAN4B6AHZAQHvAeIB6wEA8AHxAckBAPIBjgGBAQaOAfIBtgEGmwGQAdUBAJsB8gGBAQC2AfIBewEG8wF+AYwBBsoB9AGdAQb1AXoBTwEAfwH1AU8B"
"APMBogF+AQCjAaIBygEApAHWAaABAO8B6QHVAQDlAcMB5AEA5gH2Ae0BAOcB2AHmAQD3AfgB7QEA6gHfAfkBANoB6AH3AQDqAbMB3wEAzAHtAfgBAckB7gHw"
"AQDcAe4B3QEAzAH4AcYBAZ0BjwG9AQaaAYwB0wEGmwH6AfIBANUBjwH0AQBvAXkBtwEGeQHUAbcBBvIB+gGgAQDyAaABewEGeQF6AfUBAHkBbwF6AQB5AfUB"
"kQEAvgHzAZoBAJoB8wGMAQb0AY8BnQEG9QGfAZEBAPQB+wHVAQD1AX8BVwEA1QH7Ae8BAOkB9wH6AQD3AfYB+gEApAGgAbgBAJ8B9QHjAQDCAeMB9QEA4wHC"
"AZIBBPYBkwG4AQD1AeQBwgEA/AHuAekBAOQB9QFXAQDpAf0B/AEAkwH2AeUBAIoB5QHkAQCKAZMB5QEA3wH7AfkBAPYB5gHlAQftAecB5gEA9gH3Ae0BAPcB"
"6QHaAQDfAe8B+wEA8QH+AckBAO8B3wHiAQDFAf8BugEA/wHsAboBAb4B+QHzAQDVAfoBmwEA+gH2AaABANUB6QH6AQCfAeMBywEA+QGiAfMBAMoB+wH0AQCi"
"AfsBygEAoAH2AbgBAPkB+wGiAQCKAeQBVwEA7wH9AekBAO8BAAL9AQDvAQECAAIAAALxAf0BAP0B8QHwAQABAvEBAAIH/QHuAfwBB/0B8AHuAQfrAf4B7wEA"
"/gEBAu8BAAEC/gHxAQf+AesByQEAxgH/AcUBAOgB3gECAg0CAt4B7AEN/wECAuwBB+gBAgIDAg0EAsYB+AEHAwICAgUCDcYBBAL/AQADAvcB6AEH9wEEAvgB"
"BwQCBgICAgD/AQQCAgIABQL3AQMCBwUCAgIGAgf3AQUCBAIHBQIGAgQCAAcCCAIJAgYHAgkCCgIGBwILAgwCBgcCDAINAgYHAg0CDgIGBwIOAggCBg8CEAIR"
"AgYPAhICEAIGDwITAhQCBg8CFQITAgYPAhYCFQIGDwIRAhYCBhcCGAIZAgYXAhkCGgIGFwIaAhsCBhcCGwIcAgYXAhwCHQIGFwIdAh4CBh8CIAIhAgYfAiIC"
"IAIGHwIjAiICBh8CJAIjAgYfAiUCJAIGHwImAiUCBicCKAIpAgcqAisCKAIHJwIqAigCBywCLQIrAgcqAiwCKwIHLgIvAjACADACLwIxAgAyAi4CMwIAMwIu"
"AjACADQCMAI1AgA1AjACMQIANgI3AjgCADkCOgI7AgA6AjwCOwIAMwI9AjACADACPgI/AgBAAkECQgIAQQJDAkICAEICQwJEAgBAAkICRQIARQJCAkYCAEYC"
"QgJHAgBIAkICRAIARAJDAkkCAEUCRgJKAgBGAkcCSgIASgJHAksCAEcCSAJLAgBMAk0CTgIATwJNAkwCAFACTAJRAgBSAk0CUwIATQJPAlMCADUCUQJMAgA1"
"AkwCTgIAVAJSAlMCAEwCUAJPAgBVAlICVAIAVAJTAk8CAFQCTwJQAgBUAlACVgIAVwJYAlkCAFoCWAJbAgBcAlgCVwIAWAJaAl0CAFkCWAJdAgBcAl4CXwIA"
"WwJYAlwCAF4CYAJhAgBcAl8CYQIAYgJbAlwCAFwCYQJiAgBjAmQCZQIAZAJmAmcCAGQCaAJlAgBpAmoCZwIAawJjAmgCAGwCbQJuAgBkAmcCagIAagJtAm8C"
"AGcCZgJiAgBwAmwCUAIAYgJmAnECAHICaAJxAgBkAmoCcwIAcwJqAm8CAG0CbAJwAgBpAmcCdAIAaAJkAnMCAG8CbQJwAgBQAmwCaQIAcQJoAnUCAGkCdAJQ"
"AgB0AnYCUAIAZwJiAncCAHgCcQJ1AgB1AmgCcwIAeQJwAnoCAHECeAJ7AgBRAnACUAIAUAJ2AnwCAGICcQJbAgB9AnMCfgIAcwJvAn4CAH8CeQJ6AgB6AnAC"
"UQIAdAJnAoACAIACZwKBAgBaAnECewIAeAKCAnsCAHUCcwJ9AgBbAnECWgIAgwJ7AoICAHICawJlAgBrAnICYwIAYwJyAmYCAGkCbAJqAgBpAmoCbgIAhAKF"
"AoYCAIYChQKHAgCIAokCigIAiAKKAosCAIsCigJdAgCKAokCjAIAXQKNAooCAIkCiAKOAgCLAl0CjwIAkAKRApICAJMClAKVAgeUApYClQIHlwKYApkCAZoC"
"mwKcAgecApsCnQIHngKfAqACAKECogKjAgCkAqUCpgIGpgKlAqcCBqUCqAKnAgaoAqkCpwIGpwKpAqYCBjsCqgKrAgGsAq0CrgIBrQKvAq4CAYYCsAKxAgGw"
"AoYCsgIBswK0ArICAbUCtgK3Aga4ArkCugIGuwK8Ar0CBr4CvwLAAga/AsECwAIGwgLDAsQCBsQCwwLFAgbGAscCyAIGyALHAskCBsoCywLMAgbLAs0CzAIG"
"zgLPAtACAdEC0gLTAgHUAtIC1QIB0gLUAtYCAdcC2ALZAgXaAtgCrwIF2ALaAtsCBdgC2wLZAgXcAtkC2wIF2ALXAq8CBd0C3gLfAgXfAt4CtgIF4ALhAuIC"
"BeMC4QLgAgXkAuAC4gIF4gLhAuMCBboC5QLmAgXnAugC6QIJ6gLpAugCCesC7ALtAgntAuwC7gIJ7wLwAvECBu8C8gLwAgbzAvQC9QIG9AL1AvYCBqsC9wI7"
"AgGyAvgC+QIB+gL7AvwCAf0C+wL8AgH8AvoC/gIB+gL/AgADAQED+gICAwH+AgMDBAMBBQMBAwIDAQID+gL/AgEEAwMDBgMBAwP+AvoCAf0C/AIHAwH6AgED"
"CAMB/wL9AgcDAQMD+gIJAwEJA/oCCAMB/gIEA/wCAQoD/QL/AgELAwwDBAMABAMMA/wCAA0DBwMMAwACAwcDDgMABwMPAw4DAAcDAgP/AgAQAxEDEgMAEwMU"
"AxUDABYDFwMUAwAUAxcDGAMAGQMaAxYDABUDFAMYAwAUAxMDGwMAEQMQAxwDABgDFwMdAwASAxMDHgMAHgMTAxUDABwDHwMgAwAZAxYDIQMAEAMSAx4DABQD"
"GwMRAwAbAyIDEQMAEQMiAxIDACADIQMRAwAhAxYDFAMAEQMcAyADABEDIQMUAwAjAyQDJQMAJAMmAycDACYDKAMnAwAlAyQDJwMA9wIjAyUDACcDKAOyAgAp"
"AyYDKgMAKgMrAywDACYDKQMoAwArAyQDLQMAJAMjAy0DACUDJwMuAwAqAywDKQMALgMnAy8DAC8DJwMwAwAxAyUDMgMAJwOyAvkCACgDMwMpAwApAzMDNAMA"
"NAM1AywDACwDNgMrAwAtAzcDIwMAOwL3AjEDADED9wIlAwAlAy4DMgMAhgInA/kCADYDNwMtAwAjA/cCqwIAMwM1AzgDADkDJwOGAgAwAycDOQMAsgIoAzoD"
"ADoDKAM7AwAoAzMDOwMAOwMzAzwDADwDMwM4AwA1AzYDOAMAOAM2Az0DADYDNwM9AwA9AzcDPgMANwMjAz8DAEADQQNCAwBDA0EDRAMAQQNDA0IDAEUDQANC"
"AwBGA0cDSAMGRwNJA0gDBkkDSgNLAwZMA00DTgMGTgNNA08DBlADUQNSAwZTA1QDVQMAVgNXAwwDBlgDWQNaAwZaA1kDWwMGXANdA14DCV8DYANhAwlgA18D"
"XQMJYgNdA1wDCWEDYwNkAwlgA10DZQMJXANmA2cDCWUDXQNiAwliA1wDZwMJXgNdA18DCV8DYQNkAwloA2MDYQMJaQNqA2sDBWsDagNsAwVtA24DbwMFcANu"
"A20DBSYDJANxAwWFAkECQAIAcgNzA3QDB+cC6QLkAgl1A3YDdwMGeAN5A3oDBngDewN5AwZ8A30DfgMGfgN9A38DBn4DfwOAAwaBA4IDgwMGggOEA4MDBoID"
"hQOEAwaFA4YDhwMGhwOGA4gDBogDiQOGAwaIA4oDiQMGigOLA4kDBosDjAONAwaNA44DjAMGjwOQA5EDBpADkgORAwaQA5MDkgMGlAOVA5YDBpQDlwOVAwaX"
"A5gDmQMGmAOaA5kDBpgDmwOaAwabA5wDmgMGnQOeA58DBp0DngOgAwagA6EDngMGoAOiA6EDBqMDpAOlAwalA6YDpAMGpQOnA6YDBqgDqQOqAwaoA6sDqQMG"
"qwOsA6kDBq0DrgOvAwawA7EDsgMGsAOzA7EDBp2g2mbmMeCYtVT2L/eANlUaLYWWoVIxOEKXlw/284aVRhr1PQWVIzU7SxOoMQtiPludkgtUooeUUiFSpKWa"
"HAtPmq6Vi0g2Em+U3i2RtcWXOCPTxXGY3FhqmmaUP0ln/2GWGA5mxh2Z3QvjZ5yXqx79cNKX+SwbZPGUdD6YZgWUlTJebQ6aKliCwwK6GDo1efid+EtGXRac"
"vVI3TJ+dq2ER98qa01dT5hWoJQvVyWGxJhIrxcSsHkgcc3Sbu1dJCNyzKhWAez62UxsRpbinBnZjmvSkb3cvxcSjR3QRCaG0VxApqNKoZiaHwgurxCqmRm3K"
"SjQaXYyjP1EwWCali3Qz+hKjEnM4ypS5mBFQXO67pAwfYqK2yCsjTUmvFnRAJZS8qwtXZUK9e0FPeeu71x2JfMq2Iy8yfAXO6369yOy4iguxw6zOvhK6ZRXP"
"UnkKHBrH435rmkTSNQtmmlTHnzI/fbq67QntoV9YLAuQPJEBJgypeyFFHwvOYgW5FwuqQqU+f0aIeM38P1ANYqzNfn6X6L3LxXtSCQ4JVn1EBtf1OFOzWjf4"
"T3UNKEhZWxUneYz/pUHHfPjn/R0bfyEXUS3Yfq5AixbsfLv+gTdcfifqASz0ftFGDzIUfFop5Xl8GqQ5QVFJWgUxAH/cyLAa+R26f0k0xn1W50deR3hhmkJO"
"wzpGeJs523yIC1lUhHQ8Jb1G2wpvwnRaKk+HXv5O9Ar6ceNGCwsSQ05YLAujybwylzU/W8Uz/RF/Z5JHOyUGTCFbUlXGUehkaxDlZmRGBgvjp/5YAwuGoQRK"
"Xw9ApE9FSCS5wAVGfA+MxOJFqybDqtdj2AqPmgpGiwlaYApUtC2JRPFpkA10PaNS2Q8HQolTRix9rk9qzyJgpDZq4DiMVddkAEksYFVaqXXuxzlNZRBBx05p"
"Oiv2vJps4A4UyDhsHjOKbHpsWT/0ZUVZlHTYCuVdVmmx/95bpnKH+vJmRFixwOplAVm6midh9GXXMUhntS3tRihlK1cL6ldlA1YjCWNlJVASTjhlD1PZLjdm"
"Elfm/p5ozlIKON5qK0hlA2Nr0UXQQVN+GFZBLYFo4ROzXYFoHQX1UIFonhblQ4FogSdaUsGX4ROzXcGXnhblQ8GXHQX1UMGXgSdaUoFogScRt4Fo4RNqwoFo"
"HQWrtYFonhabqMGXgScRt8GXHQWrtcGX4RNqwsGXnhabqLIz0hVngE1pVRJ8nQeXVRJ8nSOcihW4gh9lVxVQhCxoW1E2jnlmUFd3nUNrJSp8nQtpAit4jb63"
"wHv4kiWwX3twnddE43gRkxhME316nXBNpipqgVFXV0m5jjaWpC16jc2Xj1Rdj4imsyAngJSwqj9KiPa07BvWgG3O7RE5gsmz0xsBgRae3BEnhGq07RsHgcZL"
"ZBvAgP5jOxH6gumWkgyEnZZpaAx/nYFqJyyFnU+Wcyt3nfLB4yN6hBOaS1d2nTudtlFnjhmVISp/nZGuIzI5jeKwSCZDjbKtIzKXm1uuPiaam8dRIzI5jaRS"
"IzKXmx9RPiY9jf1RPiaam+SmenaBnSapGnZYkmiiX2ETjcdYbXZ8nWViR1x5nWRXWnZjkjxew19djCesQFCuh+T62k8ohJzTZk0/g5pUX1HwhXYHjEwrg/4P"
"2kC4gV+viyQzgUTZfEP+goTbF0sQhUSsrEgkhmFWE0mshFxPQCRdg2lF1Xrcj6kIlHhehdit+XZSj1AJcHiIii/53X0MkAut03allLU+EXx2kN3JFH75jpY+"
"8XvHlZq8BX2wlKwW81TjgrIHEnkkinJWdHcWjwKo+HVPkI+lRFcwiOhYy1eqicqsOkwIjerO+k/vgSHZGkRtgmSb0lhTnU2vyHGpJeNCmnfkJcvKe3oqKQAA"
"AQACAAAAAAIAAwAABAAFAAYAAAUABAAHAAAIAAkACgAACwAEAAYAAAwADQAEAAAJAAwADgAADAAEAA8AAAIAAQADAAcNABAABAAAEQASABMAAAkADgAKAAAU"
"AAYAEwAAFQAUABMAAA4ADAAWAAALAAYAAQAADAAPABYAABUAEwASAAAEAAsADwAAFAAVABcABBgABgAUAAADAAEABgAAGAAZAAMAABoAGwAPAAAQABwABAAA"
"DQAdABAABhQAHgAYAAQDAAYAGAAADwAfABoAABEAIAASAAAfAA8AAQAAFgAPABsAAAwACQAhAAYPAAsAAQAAFgAiAA4ACRIAIAAVAAADABkAAAAJGwAaABYA"
"CSMAIgAWAAkfACQAGgAACAAlAAkABiYADAAhAAYFACcABgAGDAAmAA0ABgYAKAATAAcpAAAAGQAAHwABAAAACSQAHwAAAAkWABoAKgAJGgAkACoAASMAFgAr"
"AAEsABEALQAGLgAGACcABykAGQAYAAAqACsAFgAJJAAAAC8AACMAKwAqAAEkAC8AKgAALAAwABEABjEAHgAUAAQEABwABwAAJQAhAAkABiAAMgAzAAAiACMA"
"NAAAIwAqADQAADUAHAAQAAYmAB0ADQAGJwAFAAcABgAAKQAvAAATADYAEQAHLgAoAAYABxQAFwAxAAQzABcAFQAELwA3ACoAACIANAA4AAA2AC0AEQAHFQAg"
"ADMAAAoAOQAIAAYzADoAFwAEOQA7AAgABjUAPAAcAAcwAD0AEQAHEwAoADYABzsAOQA1AAcHAD4APwAHNgAuAC0ABx4AQABBAAAsAD0AMAAHPQAgABEAAC4A"
"NgAoAAdCADQAKgAAQwAqADcAACoAQwBCAABAAB4AMQABPwA9ACwABz4APQA/AAcYAB4AQQAAQwBEAEIAADkAPAA1AAcvAEUARgAJGABBACkADUUALwApAAk9"
"AEcAIAAAQAAxAEgAAUkAMwAyAAc6ADMASgAHRgA3AC8AAEEARQApAA0gAEsASQAGTAAxABcAB0gAMQBMAAdNAEoAMwAGOgBOAEwAADIAIABJAAYXADoATAAA"
"TQAzAEkABzcATwBEAAlJAEsATQAGOgBKAE4AB0MANwBEAAlLACAARwAANwBGAE8AAEEAUABFAA1RADgANAAAUgBNAEsABkoATQBSAAZTADQAQgAJNABTAFEA"
"CTgAUQBUAABVAEwATgAAVQBIAEwABzwABwAcAAdEAE8AVgAJRABWAEIAAFYAUwBCAABAAEgAVQAHRQBXAEYACUYAVwBPAAA5AFgAPAAHUABBAFkADVoAPQA+"
"AAc9AFoARwAABwBbAD4AB0sASgBSAAdYAFwAPAAHXQBeAF8AB08AVwBWAABLAE4ASgAHUABgAEUACV0AYQBeAAc5AGIAWAAHYwBiADkAB2QAZQBmAAdkAGcA"
"ZQAHRQBgAFcACWgAYwA5AAdpAF8AXgAHXwBqAF0ABwcAPABbAAdhAGkAXgAHYQBaAD4AB1EAUwBWAABLAEcATgAAQABZAEEAAGsAagBsAAZtAGQAbgAGbwBh"
"AF0AB0AAcABZAABWAHEAUQAAcgBtAHMABlAAWQBgAA1xAFQAUQAAWwA8AGsABlwAdABrAABHAHUATgAAcABAAHYABHcAcQBWAAA8AFwAawAAdQBVAE4ABHYA"
"QABVAARWAFcAdwAAdwB4AHkAAXEAdwB5AAF6AHsAcQAJXABYAHQABj4AaQBhAAZvAF0AagAHWQB8AGAAAGAAfABXAAB8AHcAVwAAYwBuAGQABn0AagBrAAZu"
"AHMAbQAGeQB4AH4ACX8AeAB3AAFhAEcAWgAAcAB8AFkAAHAAgAB8AAB8AIEAdwAJeQB+AHEACYEAfwB3AAmCAHgAfwABewBUAHEACXMAdAByAAZ8AIAAgwAJ"
"fQBvAGoAB3UAdgBVAAR6AHEAfgAJfwCBAIQAAIIAfgB4AACFAIAAcAAAggB/AIQAAIUAcAB2AACDAIAAhQAAaABuAGMAAHQAcwBrAABrAHMAhAAAhACDAGsA"
"AIMAhQBrAABHAGEAdQAAdgB1AGEAAIUAbwB9AAB2AGEAbwAAbwCFAHYAAHoAcwB7AACDAIQAgQAAegCEAHMAAIQAegB+AABoAHsAbgAAewBzAG4AAGsAhQB9"
"AACEAH4AggAAgQB8AIYADYMAhgB8AACGAIMAgQAHhwCIAIkABocAiQCKAAaLAIwAjQAGiwCOAIwABo8AkACRAAaPAJEAkgAGkwCUAJUABpMAlgCUAAaXAJgA"
"mQAHmgCXAJkAB5cAmwCYAAecAJ0AngAAnwCcAJ4AAKAAoQCiAAChAKMAogAApAClAJ8AAKYApwCoAACoAKcAqQAAqgCrAKwAAKsArQCsAACuAKoArQAAqwCo"
"AK8AALAArwCkAACrAK8AsAAArwCoAKQAALEAqACtAACyALAAswAAqACxALQAAJ8AsACkAACzALAAnwAApACoALUAAKYAqAC0AACoAK4ArAAArgCoAKoAALYA"
"twCmAAC2AKYAuAAAuQC6ALsAB7oAvAC7AAe9AL4AvwAHvwC+AMAAB8EAwgDDAAnEAMUAxgAJxgDFAMcACcgAyQDKAAHJAMsAzAABzQDOAM8AANAAzQDPAADP"
"AM4A0QAA0gDTAM0AANMAzgDNAADUANUA1gAA1QDXANYAANQA1gDYAADYANYA2QAA1wDUANUAANoA2wDcAADbAN0A3AAA3gDfAOAACeEA3wDiAAniAN8A3gAJ"
"3gDgAOMACeQA5QDmAAXBAMMA5wAJ6ADpAOoABg=="
;
static const char* CARDATA_OLDJEEP =
"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"
"BjABZAFwAQYiAVABMgEGWAFtAVkBBmUBXAFAAQFeATEBAQEATgFfAS8BAP4ADAFxAQZoAWoBbAEGaQFoAWwBBlMBcgFBAQYrATwBXQEJUAEkAWEBBmEBcwFi"
"AQBmAXQBNAEGSgF1ATMBAHYBXAFlAQFLATsBZgEATwF3AUgBDUMBeAEtAQAdAUIBHgEGJAFvAWEBBlwBdgF5AQBNAXoBPAEJLQFjAfUAAEIBewFgAQZPAVwB"
"eQEAeQF3AU8BAGEBfAFzAQY2AVQBVwEAUwFXAXIBBgsBfQFDAQD3AH4BLgEGLgF+AX8BBloBZwFtAQaAAXcBeQEAegFMAWMBCSEBLgF/AQZvAXwBYQEGLAGB"
"AV4BCXsBggFgAQaCAWQBYAEGrAD9AH0ABlcBgwFyAQZzAYQBYgEAhQFYAWIBBnQBWwE0AQZIAYEBLAEJSgGGAXUBAHYBgAF5AQFIAXcBgQENWgGHAUoBAIgB"
"ewFBAQZlAYkBdgEAMwGJAWUBAAsBHAF9AQAhAX8BIwEGJAGKAW4BBnsBQgFBAQZzAXwBhAEGhAGLAWIBAKwAcAH9AAaMAXgBQwEA9wCNAX4BBqwAMAFwAQZ9"
"AYwBQwEABgEZAUYBAHgBjgFjAQB/AY8BIwEGhQGHAW0BBnUBiQEzAQB3AZABgQENXwFOAZEBAI0B9wCSAQZyAZMBiAEGiwGFAWIBBocBhgFKAQCJAYABdgEB"
"gQGUAV4BCTEBXgFOAQBrAX0BHAEAQQFyAYgBBgYBRgE2AQZYAYUBbQEGWwF0AZUBBm0BhwFaAQZ+AZYBfwEGjQGXAX4BBmYBOwF0AQA7AZUBdAEAKwE7AUsB"
"AIwBmAF4AQBeAZEBTgEAkQGZAV8BAJoBgAGJAQF3AYABkAEALQF4AWMBAJcBlgF+AQaKASQBIwEGjwGbASMBBo8BlgGbAQacAYwBfQEAfQFrAZ0BAI8BfwGW"
"AQYjAZsBngEGmwGfAZ4BBo0BkgGgAQagAZcBjQEGfAFvAYQBBqEBhgGHAQBMAXoBTQEJmQFrAV8BAP4AcQGiAQajAVQBRgEGgQGQAaQBCf4AogFVAQZbAaMB"
"RgEAdQGlAYkBAJIB9wCmAQaSAacBoAEGqAGnAZIBBlQBqQGqAQaUAYEBpAEJYwGrAXoBAI4BeAGYAQD9AKwBDAEGigEjAa0BBqcBrgGgAQZvAa8BhAEGgwGT"
"AXIBBrABhAGvAQaDAVQBqgEAVwFUAYMBBpUBsQFbAQayAbMBXgEAhAGwAYsBALQBhQG1AQaFAbQBhwEGoQGHAbQBBjsBsQGVAQClAZABgAEAqwFjAY4BAJEB"
"XgGzAQCZAZ0BawEA9wC2AbcBBvcAtwG4AQa4AaYB9wAGggG5AWQBBpIBpgGoAQaKAboBbgEGgwGqAZMBBoUBuwG1AQa1AbwBtAEGogFxAb0BBrYBvgG3AQa+"
"Ab8BtwEGvwHAAbcBBsABwQG3AQaeAcIBrQEGIwGeAa0BBmQBuQFwAQbDAW4BugEGwwFvAW4BBrABxAGLAQaFAYsBuwEGpQGaAYkBAJoBpQGAAQA8AXoBXQEJ"
"qwHFAXoBAJgBjAGcAQCdAZwBfQEAnQHGAZwBAMcBDAHIAQb3AP4AtgEGtgH+AFYBBsABvwHJAQbJAcoBwAEGygHLAcABBsEBzAG4AQBxAc0BvQEGzgG8Ac8B"
"BM4BtAG8AQTQAaUBdQEApQGkAZABAJkB0QHSAQCZAZEB0QEAvwHTAckBBskB1AHKAQbAAdUBwQEGtwHBAbgBAK0B1gGKAQbXAa4BpwEGrwFvAcMBBsQBuwGL"
"AQBUAaMBqQEAOgFdATsBAJQBpAHYAQmOAcUBqwEA2QGdAdIBAJ0BmQHSAQDTAdQByQEApwGoAdoBALUBzwG8AQZbAdsBowEAzgGhAbQBBl4BlAGyAQmzAbIB"
"lAEAjgGYAdwBAKwB3QEMAQbVAcwBwQEGuAHMAdoBAHABrAH9AAamAdoBqAEAegHFAV0BAMUBjgHcAQDRAZEBswEA3QHeAQwBBgwB3gHIAQaIAbkBewEGqQHf"
"AaoBAOABzwG1AQA7AeEBsQEA4gHQAXUBALMBlAHjAQCzAdIB0QEJpwHaAeQBAOUBsAGvAQDEAbAB5QEAowHfAakBAOYB5wHOAQDOAecBoQEAoQHnAYYBAIYB"
"4gF1AQCUAdgB4wEAmAGcAegBAOkB6gGdAQncAZgB6AEAxgHoAZwBAMYB6gHrAQnGAZ0B6gEAVgHsAbYBBu0BvgG2AQbUAe4BygEGywHKAe8BAMsB8AHAAQbA"
"AfAB1QEA1gHxAYoBBp8B8gGeAQbzAeQB2gEAuQH0AXABBtcBpwHkAQafAfUB8gEA3gH2AcgBBr0B9wH4AQb5AfoB7AEA7AH7AfwBBv0B3gH+AQa9Ac0B/wEG"
"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"
"Al0C5QEAWAJnAmsCAGMCMgJnAgBeAmUCaAIAXgIsAmUCAN8BbQIyAgAyAm0CZwIASQJoAmkCAGsCZwJhAgczAlkCUAIAVQJaAmkCAFACWQJrAgBtAmACagIA"
"3wFgAm0CBWgCVgJpAgBkAlACXAIAZwJtAmoCAGcCagJhAgdbAmQCXAIAagJgAjYCAEgCaAJlAgBIAmICaAIAYQLrAWsCAFICVQJpAgBWAlICaQIAYgJWAmgC"
"AFAC6wFcAgA5AkICbAIAPAIyAmMCABkCPAJjAgDlATkCbAIAVgJXAmYCAGYCTQJWAgBuAm8CcAIGbgJwAnECBm4CcQJyAgZuAnMCdAIGbgJ0AnUCBm4CdQJ2"
"AgZuAnYCdwIGbgJ3AngCBm4CeAJ5AgZuAnkCbwIGegJ7AnwCBnoCfQJ+AgZ6An4CfwIGegJ/AoACBnoCgAKBAgZ6AoECggIGegKCAoMCBnoCgwKEAgZ6AoQC"
"hQIGegKFAnsCBoYChwKIAgaGAokChwIGhgKKAokCBoYCiwKKAgaGAowCiwIGhgKNAowCBoYCjgKNAgaGAo8CjgIGhgKQAo8CBoYCkQKQAgaSApMClAIGkgKV"
"ApMCBpIClgKVAgaSApcClgIGkgKYApkCBpICmgKYAgaSApsCmgIGkgKcApsCBpICnQKcAgaSApQCnQIGngKfAqACBp4CoAKhAgahAqACogIGowKiAqQCBqMC"
"pAKlAgalAqQCpgIGpwKoAqkCBqkCqgKoAgapAqsCqgIGqwKsAqoCBqsCrQKsAgatAq4CrAIGrQKvAq4CBrACsQKyAgaxArMCsgIGtAK1ArYCBrUCtwK2Aga3"
"ArgCuQIGuAK6ArkCBrgCuwK6Aga7ArwCugIGuwK9ArwCBr0CvgK8Aga9Ar4CvwIGwALBAsICBsMCxALFAgbGAscCyAIGyALJAsoCBssCzALNAgbOAs8C0AIG"
"zwLRAtACBs8C0gLRAgbSAtEC0wIG1ALOAtUCBtYC1wLYAgbXAtkC2AIG1wLaAtkCBtcC2wLaAgbXAtwC2wIG3ALdAtsCBtwC3gLdAgbeAt8C3QIG3gLgAt8C"
"Bt4C4QLgAgbhAuIC4AIG4QLjAuICBuMC1wLWAgbjAtwC1wIG4wLeAtwCBuMC4QLeAgbkAuUC5gIG5gLlAucCBuYC5wLoAgbmAugC6QIG6QLoAuoCBukC6gLr"
"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"
"UAAAbABxAG8ABm0AcgBqAAZkAGMAcwAASgBoAE4AADwAdABCAAZqAHIAawAATgB1AF8AAGMAXgB2AABXAF0AdwAAVgB4AGIAAF4AZwB2AAB5AG4AZAAJbgB6"
"AFsAAFQAewBpAAY6AGsAfAAGcAB9AG0ABn4AfwB0AAaAAGYAgQAAewBUAIAAAIEAZgBaAAZwAG8AfQAAbQB9AHIAAGkAewBsAABvAHEAfQAARQBZAFgABnsA"
"fQBsAACBAHsAgAAAbAB9AHEAAEUAZQBZAACBAH0AewAAcgB8AGsAAHIAggB8AAA8AF8AfgAGXwCDAH4AAHwAhACFAABOAGgAdQAAdwB4AGEAAGEAeABWAARc"
"AGIAdgAAdgCGAGMACXoAhwBbAAA8AH4AdAAGggByAH0AAH0AgQCIAACJAHgAZQAAdQCDAF8AAFsAigBSAACHAHoAbgAJfgCLAH8AAFwAdgBnAABzAGMAhgAJ"
"UwBmAFQAAIgAggB9AACEAIkAhQAAiQCEAHgAAIwAcwCGAAlzAIwAjQAAZQB3AFkAAFcAdwBhAAB5AGQAcwAAVABmAIAAAHcAXQBZAACKAGgAUgAAigBbAIcA"
"AG4AeQCHAAl5AHMAjQAAjQCIAI4AAIQAdgBiAAB8AIIAhAAAiwCPAH8AAH4AgwCLAACIAI0AggAAeACEAGIAAIoAhwCQAACNAIwAhgAAZQB4AHcAAI4AjwCL"
"AACHAHkAjQAAdQBoAIoABY0AjgCLAACHAIsAkAAAiwCDAJAAAIMAdQCQAACNAIYAggAAdQCKAJAAB40AiwCHAAB2AIQAggAAhgB2AIIAAJEAkgCTAAaUAJUA"
"lgAGlwCYAJkABpoAmwCcAAadAJ4AnwAGoAChAKIABqEAowCiAAahAKQAowAGoQClAKQABqUAoQCgAAamAKcAqAAGqACnAKkABqgAqQCqAAaqAKkAqwAGqgCm"
"AKgABqwArQCuAAatAK8AsAAGrQCxALAABrEAsgCwAAaxALMAsgAGsQC0ALMABqwAsQCtAAasALQAsQAGtQC2ALcABrUAtwC4AAa4ALcAuQAGuAC5ALoABrgA"
"ugC7AAa4ALsAvAAGvAC7AL0ABrwAtQC4AAY="
;
static const char* CARDATA_PICKUP =
"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"
"AA1CAJgAKgAAqABKAEkACZcAlQCCAABuAE0AhQAGMACpAHQABkMAowAzAAajAJoAMQAGRAB3AE4AAKoAkABFAAakAKsAIgAAkgB8ACQABpwAqgBFAAYiAKwA"
"UQAGZgCdACMADZ4ArQBpAAR+AGgApgAFagBrAKYAAJUAoQA8AACDAEcAogAJQgCuAJgAAHIAhwAvAAZNAJkAhQAGcwCpADAABq8AjgCwAABkALAAjgAAsQBP"
"AJsABiIAqwCsAAY3ALIAgAAAQAAoALMAAKcAnwCgAA2nAKAAbQANlQC0AKcADZQArgBCAABCAEsAlAAAlQCXALUAAEkAlgCoAA1wAJkAcQAGcQCZAIoABo8A"
"mgCkAAB3AEQAjgAAtgB8AJIABpIAtwC2AAa3AJIAfQAAnACRALgAALkAugC7AAC7ALwAuQAGugC9ALsABrkAvgC/AAa5ALwAvgAGwADBAMIADbsAwwC8AAbA"
"ALQAwQANkwDEACgACcUAnwCnAA3FAKcAxgANvADHAL4ABsAApwC0AA2fAL4AawAAvgDHAGsAAEEAQADIAABBAMgAPwAArgDJAJgAAKIAlgCDAAnJAKIAmAAA"
"cwCIAKkABoYAhQCZAAZyAMoAdAAGhwByAHQABnQAygCKAAZ0AKkAywAGiQCQAFoABkMAigDKAAavAHcAjgAAtgDMAHwABrcAzQC2AAayADcApQAAUwBRAM4A"
"DVEArADOAA1mAM8AnQANTwCxAH8ABmYA0ADPAA3NALcAfQAGzgDRAFMADa0A0gDTAAS5AL8A1AAAfQDTAM0ABlMA0QCeAA1pANMAfQAGrQCeANEABNQAvwDV"
"AAa/AL4A1QAGvgDFANUABsYAwADCAA3GANYAxQAN1wDFAL4ABp8AxQDXAAbAAMYApwAN1wC+AJ8AANgAtACVAA1aANkAWAAGigDaAHEABtsAigBDAAaqANkA"
"WgAGmQB3ANwABkMAdwCZAAapAN0AmgAGmQDbAEMABsoAywCjAAajAEMAygAGywCpAKMABpoAowCpAAbeAN8AzAAGiQCbAJAABq8A3AB3AAbeAOAA3wAGkACq"
"AFoABrYA3gDMAAa4AJEA4QAAfgDhAJEAALEA4gB/AAa6ALkA1AAG4wDQAGYADa0A0QDSAAStANMAaQAG4gDjAH8ADWYAZwDjAA1/AOMAZwANxQDWANUABsIA"
"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"
"AUwBCXIBLAEzAQCzAGoBIQEAQgFzAUsBCTwBNQFaAQYQAXQBZgEAOAFiAXUBAHYBbwFuAQZvAXYBcAEGXwFsAUMBBnEBdwE9AQlCAXgBcwEJdgFuAXABBmYB"
"eQFlAQYsAXoBagEA+QA7AUcBBkIBTQF4AQlMAWsBcQEJIgEAAUYBADgBewFGAQA1AWQBWgEGfAEQAUQBBk4BaAFjAQAuAUgBFwEGbAFlAXkBBloBfQFWAQZ0"
"AX4BZgEAKwF/AR8BDXIBMwFWAQB6ASwBcgEAgAE5AScBBnkBgQFsAQZjAYIBeAENFwGDAYQBBoUBRQHwAAB1AXsBOAEAbAGGAUMBBocBgQF5AQB5AWYBhwEA"
"IQFqAU4BAHgBTQFjAQ13AUoBPQEJOQFkAToBBkMBiAFIAQZsAYEBhgEGawFGAXEBAEsBcwFiAQBeAYkBNgENiAFQAUgBBhcBigGDAQZDAYYBiAEGOwFhAUcB"
"Bl0BNgGJAQ2FAYsBRQEAaQGCAWMBDUQBRQF8AQBFAYsBfAEAjAGFAfAAAEYBjQFxAQB7AY0BRgEAcgFWAX0BAGcBQAEfAQAQAXwBdAEGTgFqAWgBAI4BcwF4"
"AQmPAUcBYQEGkAFKAXcBCY0BdwFxAQCNAXsBkQENOQGSAWQBBpMBWgFkAQZ+AZQBZgEAZwGVATsBAIwB8ABHAQCWAWkBaAEAbQGKARcBBpMBZAGXAQaHAZgB"
"hgEGjwGMAUcBAIIBaQGWAQ2OAZkBcwEJWgGTAZoBBnIBfQGaAQCbAXwBiwEAaAGCAZYBAJABdwGcAQmZAWIBcwEAJwEOAZ0BBoYBngGIAQBAATsB6wAAWgGa"
"AX0BBsIA7QArAQ3tAF0BKgENmQF1AWIBACcBnQGAAQZQAYgBnwEAnQGgAYABBnsBdQGhAQCiATkBgAEGUAGfAW0BBqMBpAGAAQalAZ4BhgEAgQGHAYYBBu8A"
"XQHtAA2ZAaYBdQEA9QACAZ0BBoABpAGiAQanAaABnQEGqAGpATkBBn8BZwEfAQByAaoBqwEAeAGCAY4BDYIBrAGOAQ2KAa0BgwEGLQEXAYQBBoQBrgEtAQau"
"Aa8BLQEGrwGwAS0BBjkBogGoAQYUAS0BsQEGowGAAaABBoYBmAGlAQaHAWYBmAEAZgGUAZgBAHIBqwF6AQChAZEBewEAkgGXAWQBBg4B9QCdAQaSATkBqQEG"
"ngGfAYgBAKcBsgGgAQazAWcBfwEGKwGzAX8BDZEBdwGNAQCfAbQBbQEGsQG1ARQBBrQBnwGeAQaTAZcBtgEGtwGzASsBDbcBuAGzAQa3ASsBuQENugG3AbkB"
"DZEBuwG8AQChAbsBkQEAtQEVARQBBgIBpwGdAQaYAb0BpQEGOwGVAWABALcBugG4AQZ8AZsBdAEGvgFoAWoBAKYBvwGhAQBtAbQBwAEGsQHBAbUBBpIBqQHC"
"AQalAb0BngEGkwHDAcQBBpoBxAFyAQByAcQBqgEAuAFnAbMBBioBuQErAQ2mAaEBdQEAigHFAa0BBj4BAgHaAAa9AbQBngEGxgHEAcMBBioBxwG5AQ3IAYwB"
"jwEGvgGCAWgBAJIBwgGXAQaxAckBwQEGsgHKAaABBsMBkwG2AQaaAZMBxAEGxAHLAcwBAH4BdAGUAQBnAbgBzQEAYAHOAWEBBrkBxwG6AQ2PAWEByAEAzwGO"
"AdABCXcBkQG8AQDRAcIBqQEGlwHCAbYBBpgB0gG9AQbTAWcBzQEAKgFdAdQBDdUBqwGqAQSbAdYBdAEG1AFdAYkBDboB1wG4AQCsAdABjgENnAF3AbwBALsB"
"oQHYAQCKAW0BwAEGtQHZARUBBj4B2gECAQYCAdoB2wEGAgHbAacBBqcB2wHcAQa1AcEB3QEGdAHWAZQBAM4BYAGVAQbHASoB1AEN1wHNAbgBANcBugHHAQ3O"
"AcgBYQEGXgFKAd4BDZAB3gFKAQ2OAc8BmQEJzAHVAaoBBIsB3wGbAQDIAc4B4AEGegHhAWoBAIwByAHgAQDiAawBggEAnAHeAZABAOMBpgGZAQDYAaEBvwEA"
"3AGyAacBBsUB5AGtAQa9AeUBtAEGqgHEAcwBAJUB0wHmAQDTAZUBZwEA1wHHAecBDeAB6AGMAQDoAYUBjAEGggG+AeIBAMAB6QGKAQY+AeoB2gEGwAG0AesB"
"BhUB7AE+AQbdAcEB7QEGwgHRAe4BBrQB5QHrAQbtAewBFQEGFQHdAe0BBu8B8AHxAQbyAe8B8QEG8wHFAesBBvMB9AHFAQb0AeQBxQEG9QHtAcEBBu0B9QH2"
"AQDCAe4BtgEG9wHDAe4BBu4BwwG2AQbLAcQBxgEA+AH5AfoBBvsB/AH9AQ3+Af8BAAIEzgGVAeYBAAECAgIDAg0DAgICBAIGBAICAv8BBP8BAgIAAgQFAgYC"
"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"
"AgZHAnUCZQIAQwIgAkICBnECdgIyAgZ2AnwCMgIGdAJFAmcCBnYCdwJ8AgBoAmkCdwIGdwJpAnwCDTACeAJrAgB6AnsCVgIAawJ4AnUCAHsCegJ9AgZ9AlsC"
"ewINHgIgAnQCBnUCeAJdAg1bAn0CQAINZQJ1Al0CDXICfgJ9Ag1yAnMCfgINcwJFAn4CBmkCfwJ8Ag1wAn0CegINXQJ4AisCDTICPwJ5AgYyAnkCcQIGcAJy"
"An0CDTQCMwIyAgZ/Al0CNAINQwJAAn4CBiACQwJ+AgZ+AnQCIAIGRQJ0An4CBmUCfwJvAg18An8CNAINfwJpAm8CDV0CfwJlAg19An4CQAINMgJ8AjQCBoAC"
"gQKCAgaAAoICgwIGgAKDAoQCBoAChQKGAgaAAoYCgQIGhwKIAokCBocCiQKKAgaHAosCjAIGhwKMAo0CBocCjQKIAgaOAo8CkAIGjgKRAo8CBo4CkgKRAgaO"
"ApMClAIGjgKQApMCBpUClgKXAgaVApgCmQIGlQKaApgCBpUCmwKaAgaVApcCmwIGnAKdAp4CBp8CngKgAgafAqACoQIGogKjAqQCBqMCpQKkAgajAqUCpgIG"
"pgKlAqcCBqcCqAKlAgaoAqkCqgIGqwKsAqkCBqsCrAKtAgauAq8CsAIGrwKwArECBrICswK0AgayArUCswIGtQK2ArMCBrcCuAK5Aga3AroCuQIGugK5ArsC"
"BroCvAK7Aga9Ar4CvwIGvgLAAr8CBr4CwQLAAga+AsICwQIGwgLDAsECBsICxALDAgbCAsQCxQIGwgLGAsUCBsYCxwLFAgbGAsgCxwIGxgLJAsgCBskCygLI"
"AgbJAssCygIGyQLMAssCBskCzQLMAgbJAs0CzgIGzgK+Ar0CBs4CwgK+AgbOAsYCwgIGzgLJAsYCBs8C0ALRAgbPAtIC0AIGzwLSAtMCBtMC0gLUAgbTAtQC"
"1QIG0wLVAtYCBtMC1gLXAgbTAtcC2AIG2ALXAtkCBtgC2QLaAgbYAtoC2wIG2wLaAtwCBtsC3ALdAgbbAt0C3gIG3gLdAt8CBs8C0wLYAgbPAtgC2wIGzwLb"
"At4CBuAC4QLiAgbgAuMC4QIG4ALkAuMCBuAC5QLkAgblAuYC5AIG5QLnAuYCBuUC6ALnAgboAukC5wIG6ALqAukCBuoC6wLpAgbqAuwC6wIG6gLtAuwCBuoC"
"7gLtAgbuAuUC4AIG7gLoAuUCBu4C6gLoAgbvAvAC8QIG7wLwAvICBu8C8gLzAgbzAvIC9AIG8wL0AvUCBvMC9QL2Agb2AvUC9wIG9gL3AvgCBvgC9wL5Agb4"
"AvkC+gIG+AL6AvsCBvgC+wL8Agb8AvsC/QIG/ALvAvMCBvwC8wL2Agb8AvYC+AIG/gL/AgADBv4CAQP/Agb+AgIDAQMGAAMDAwQDBgADBAP+AgYCAwUDBgMG"
"AgMGAwEDBv8CAQMHAwb/AggDCQMG/wIJAwADBgADCgMDAwYDAwoDCwMGAwMLAwwDBgMDDAMEAwb+AgQDDQMG/gINAwIDBgIDDQMOAwYCAw4DDwMGAgMPAwUD"
"BgUDDwMQAwYFAxADEQMGBQMRAwYDBgYDEQMSAwYGAxIDAQMGCAMTAwkDBgkDEwMUAwYJAxQDAAMGAAMUAwoDBtKKijUmOXuNmi0pQ8CIO018UYWKCDd40FuT"
"81bdtV2CI0mesHqN5xtlP/aMlFPcRteYb0qcbCWL+j26ai+gkmk6KpqQnVQiJhySiFVSPqOMsRv11CiU+FpZAGuNUUyFl0KPXTkGj5KTZloLDdKaci3ngxmc"
"Qhz8o5GfyVv41FKYflZaiKylIz7bs5WqEjaBSE2dLTpNVN2Y+iqTyAijw04Xhlqd1AzoPqaiYA7UG4iOPAwY3PqkWjtxhtGV3lrq4Zm6kgnGxVuiwgog++Om"
"2g4p/u+lCRi40W2jhTdTc5m2YV0cS/KoN3W7DImnk3at44GlwQ6O58Sj5hl+QJSdsjH0ZLOjdVdHr3a35Suhe0qjt17HPc236gqORJq8a04xevqmHHQT/7ek"
"igwAc9TnEjA/hIOktDmxjMOnLFdtjRKozDrzpPKlJke9sm2t4jh10hCxl3VnJPeokA4aGkuo9w5E7IfB+wlfh+Gfugsun0C6yTpBwgO2T3KD18D7/3/+DCK+"
"GzqRrB27owpJo4RSFQ01HypLeQoByH9bEw937Pz9NDifpbX1STU1fEPRiyScg+z/gF6wRb9I9QqVZZe7zAq2ZJhcvg5A+wT1ozZAoK45lAkLhf9P9ThV09Zf"
"+U3ehTj+7Xvk2Z3+4HbxKvwdGF+OUU07LS5HfVZaI1wP1BZN5QpqeqZRwDoSrDRcJzrGjD0bKjGBhA838yOvg9BI53KQ1lNfujuQhlJk71qUQvAyBn1F3zlK"
"w0/1eMJax1cyjI1e2Q3Mpy1GRwvSqklVXzOzdylq6y5AhTVUt3FKI6ZIIA7FR+BenAzmP75UIkSGsmlZ3HMsC75XVnQd4+x2QErdZ1FYnHQzAANipgpi58xx"
"FjeJcE1qlleK0c93u1DMRFtfBg4Q+udczRc/z9tdnht6QKpkhTNPZaVihFYgsY5mJmLqLzVszVjf/hxrAVqm4fpv+1PPJ1JiWjz/USp4eyMzpBFyTlU0PpBz"
"Xwyl2WVuh1PulDJrwFmzDM5y+RvJPlZxkzDKQmxb0z44s7F6PE18r3B2izrq0NNxVxy31hxsGTB8yYx5Dky+T6ty4kr2l6l3yFNJtht1STaJOfl5sRIsTPl5"
"0CqmV/l5jw5IXPl5sRJiwvl5jw5Gsvl50CrotgyI/gqrVwyIPydMXAyIGyMxTA+IsRJlwg+I0Crrtg+Ijw5JsptUsCzcaiKkCDmUWVh8vi1JXAhP3zgMVh5g"
"SyCqQFh9xjBrTc2j5hq4QMxH5hq4QM1Y4xrxajy66ho9a6NdNyDDap5d5hoWa32r0iRSaM+ohjuIWoiEQi4YXouyhDsJUxKICDH9Simq2Br3PzhMyBoTza1H"
"zjkdwOh1rSWaxjl7CDR0qcVaqCvIpJRV/BrmoTy66xr+zUWIZCBxzjJe5hrWzU+6DTlLwueGfztLuaGT7izVpiSt5Rqwonb1ElJ6eyUzETDifXM/mDzVez2x"
"ezT/f8+1mkl1eOhWrD4E0l5TdkZu1MtLjkr0d/SpYjyu0spTuTeoeQAAAQACAA0DAAQABQANBgABAAAAAAcAAAACAA0IAAIACQAACgALAAwADQMADQAOAAAP"
"ABAABQAGBgALABEAAAgABwACAAALAAYAAAAABQASABMABg0ABgAOAAAAAAwACwAADAAAAAcADQ4ABgARAAAEAAMAFAAABAAVAAUABhYABQATAAYXAAIAAQAN"
"EgAFABAABgIAFwAYAA0PAAUAFQAGAwAZAA0ABgUAFgADAA0aABAADwAFGwAcAB0ABhoAHgAQAAUCABgACQAGDwAVABoAAA4AHwADAAAdACAAIQAGGwAdACEA"
"BhwAIgAdAAYZACMADQAGJAAIAAkAABQAAwAfAAAMAAcAJQAADgARACYADQ4AJwAfAAkdACIAKAAGAQAGACkABhgAKgAJAAYEACsAFQAADQApAAYABgQAFAAr"
"AA0nABQAHwAAIwApAA0ABioALAAJAAYWABkAAwANEQALAAoACQwALQAKAAAhAC4AGwAGCQAsACQAAAcACAAvAAAlAC0ADAAADgAmADAADSEAKAAiAAYxACwA"
"KgAGEgAQADIAADMANAA1AAY1ADQAKwAGNgAUADcADTYAKwAUAA0rADQAFQAALQA4AAoAAAoAJgARAAkgAB0AKAAGIQApABwABi4AIQAcAAYcACkAIgAGHAAb"
"ADkABiMAIgApAAYrADYANQAGOgAiACMABhIAOwA8AAYoADoAIwAGOQAbACkABhMAEgA8AAYeADIAEAAANwA9ADYADScAPgAUAAA6ACgAIQAGIQA5ACkABicA"
"DgAwAAkKADgAJgAAMAAmAD8AADoAIQAgAAYhACIAOQAGNgA9AEAADSgAIwAgAAYwAD8AJwAAPAA7AEEABiIAQgA5AAYcADkALgAGIAAiADoABiYAOAA/AAA1"
"ADYAQAAANQBAADMAAC8ACAAkAAQgAEMARAAGIgAgAEQABj0ANwBFAA0kAEYALwAARwA7ABIABkgALQAlAA0HAC8AJQAAEgAyAEcABi4ASQBKAAYkACwARgAA"
"RABLACIABkAATAAzAABHAE0AOwAGNwBOAEUADRoATwAeAAYnAD8AUAAALQBIADgACVEAOABIAAk+ACcAUAAAUQA/ADgAAC8AUgAlAABAAD0ARQAATABAAEUA"
"BkEAQwAgAAYsAFMARgAGUAAUAD4ACRQAVABOAA1KAEkAMQAGJQBSAEgADTEAVQAsAAZMAFYAVwAAHgBYADIAABoAFQBPAAAyAFkARwAAUABaABQACR4AWwBY"
"AABHAFkATQAGSABcAFEACTMAVwA0AAY/AF0AUAAALABVAFMABjsATQBBAAZZADIAWAAGTgBWAEUADVMALwBGAABeAC8AUwAAXwAVADQAAC8AXgBSAABcAEgA"
"UgANWgBUABQACUkAVQAxAAZBAGAAYQAGVwBfADQABl8ATwAVAABQAF0AWgAAVgBMAEUAAB4ATwBbAAZiAFMAVQAGTQBgAEEABlgAYwBZAAZcAGQAUQAJXgBc"
"AFIAAGQAPwBRAABMAFcAMwAALgBCAEkABkIAZQBJAAYUAE4ANwANUwBiAF4AAEMAQQBhAAYuADkAQgAGQgBmAGUABmcAVgBOAA1kAGgAXQAAZABdAD8AAEsA"
"QgAiAAZjAE0AWQAGWgBdAGkAAGoAXgBiAARpAF0AawAAQwBsAEQABm0AagBiAARaAG4AVAAAawBdAGgAAFwAXgBvAABLAHAAQgAGRABxAEsABk0AYwBgAAZm"
"AEsAcgAGVQBJAGIABnMAYgBJAAZWAGcAdAAGZwBUAHQADVwAdQBkAABrAHYAaQAJbABwAEQABkIAcABsAAZDAHEAbAAGcQByAEsABk4AVABnAA10AF8AVwAG"
"dABXAFYABloAaQB3AABmAHAASwAGWgB3AG4AAGQAeABoAAl5AG0AcwAGaQB2AHcACXgAZAB1AAloAHYAawANegBgAGMABlgAWwBjAABuAHQAVAANXABvAHsA"
"AHEAfABsAAZEAHwAcQAGRABwAHwABl8AdAB9AABoAHgAfgAJfABCAGwABmYAfABwAAZ/AIAAcgAGbQBiAHMABoEAegCCAAZPAGMAWwAFagBvAF4AAIMAbgB3"
"AABoAH4AdgANfABmAEIABoQAcQCFAAaGAHkAgAANXwB9AE8AAHYAfgB4AABcAHsAdQAAcgBxAH8ABoIAhQCBAA1jAH0AhwAFYwBPAH0ABXQAbgCIAABxAIQA"
"fwAGiABuAIMAAHUAewB4AA2DAHgAhAAAiQB/AHgAAIkAewBvAA13AHYAgwAAeACDAHYAAHQAiAB9AABqAIYAbwAAeAB/AIQAAHoAYwCHAAZ4AHsAiQAAhwB9"
"AIIABokAgAB/AACGAG0AeQAGfQCIAIIABoQAhQCDAABvAIYAgAANggB6AIcABokAbwCAAA1qAG0AhgAGgwCCAIgADYUAggCDAA2KAIsAjAAGjQCOAI8ABpAA"
"kQCSAAaTAJQAlQAGlgCXAJgABpYAlwCZAAaZAJoAmwAGmgCcAJ0ABp0AlgCeAAadAJkAlgAGnQCaAJkABp8AoAChAAafAKIAoAAGnwCiAKMABqMAogCkAAal"
"AKYApwAGnwCjAKUABp8ApQCnAAaoAKkAqgAGqQCrAKoABqkArACrAAatAKkAqAAGrQCsAKkABq4ArwCwAAauAK8AsQAGsQCvALIABrEAsgCzAAa0AK4AsQAG"
"tACxALMABrUAtgC3AAa1ALgAtgAGtQC5ALgABrcAugC7AAa3ALsAvAAGtwC8ALUABrkAvQC4AAa2AL4AtwAGtwC+ALoABg=="
;
static const char* CARDATA_SPORTSEDAN =
"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"
"RgBrAIgACWsAcwCIAAk4AFwAdgAGiQCKAEgABkoAiQBIAAZfAGoAdAAAUABaAIsABkAAgACMAAAcABsAWwAGVgCNAH0ABooARABIAAZlAFIAjgAJhABlAI4A"
"AHUAegCPAAlzAIYAiAAAbQCQAEIABmoAdgB0AACRAFcAZAAGgQCSAIIACZIAhACCAAZYAHkAkwAAkwB5AFEAAJQAUwA+AACFAIYARQAAbgCVAIAACYAAlgCM"
"AA1AAIwAVAAAigCJAJcABlYAVwCNAAaYAHIAiwAGWACTAGQAAJkAkwBRAAaSAJoAhAAAmwCGAIUAAJwARgCIAAmGAJ0AiAAATQCeAGcABp8AfwBdAAagAG0A"
"YQAGmACgAGEABlEARAChAAZhAHIAmAAGcQCiAIEAAKEAmQBRAAZSAHEAgwAJUgCDAI4ACYMAhACOAAajAJQAPgAAlQCWAIAACYwAlgCkAABGAJwApQAJpgCh"
"AEQABngApwCoAAB6AEYApQAJoACQAG0ABnAAfwCfAAZ+AHAAnwAGpgCKAJcABqYARACKAAaBAKIAqQAAZQCEAKoAAKsAmwCFAA1UAKcAeAAAbgCHAJUACawA"
"fABbAAZsAK0AXAAGdACuAHEAAK4AogBxAABmAKMAPgAAhwCPAJUACWwAaQCvAAaBAJoAkgAJnACIAJ0AAI8AhwB1AAlvALAAsQAGrgB0ALIABKsAnQCGAAmW"
"AJUApAAAegClALMACY8AegCzAAlcAK0AdgAGmwCrAIYACbQApQCcAABJALUAbwAGsgB0ALYABLcArgCyAAS4ALkAmgAGhACaAKoAAKQAVACMAACVAI8AugAA"
"uwC2AHQABHQAdgC7AABlAKoAZgAAugCkAJUAALQAswClAACpALwAuAAJuACaAKkABoEAqQCaAAmnAFQAvQAApAC9AFQAAJ0AtACcAACPALMAugAAsgC2ALcA"
"BoUAlACrAA2nAL4AqAAAugCzAL8AALQAvwCzAABbAMAAwQAGwgCwAG8ABqIAvACpAAC8ALkAuAAGwwCjAGYAAK8AwQBsAAZsAMEArQAGwgDEAMUABr0ApADG"
"AA3HAMEAyAAGtQDJAG8ABpoAuQCqAAC+AKcAygAAygCnAL0ADcoAvQDGAA2kAMsAxgANTADAAFsABswArgC3AATMAKIArgAElACFAFMAAM0AvgDKAADKAMsA"
"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"
"AQk0AUsBNgEAPAE0ATYBAFMBVQFcAQZXAV0BWAEGVQFeAVwBBl8BWAFdAQZBAUIBWwEAMgFgAUYBCTkBYQFMAQkzAWIBIAEGKwE7AUsBBmMBZAFlAQZcAV4B"
"ZgEGXgFnAWYBAB8BOAEcAQAPAQ0BRAEAYQE5AR0BACIBaAE7AQYHAWkBSAEGVwFZAWoBBmsBZAFjAQZeAVUBbAEGbQFuAW8BAF4BbAFnAQBwAWYBZwEAbgFx"
"AW8BAAkBNwFDAQY4AT0BHAEAWwFyAUEBDVoBcgFbAQluAXMBcQEAPwFBAXIBAB0BdAFhAQBHAcgAdQEGaQEHAe4ABmoBdgFXAQZ3AWQBawEGUgFUAXgBBlcB"
"dgF5AQZ6AQ8BRAEAewFPAVEBBjsBfAFLAQZLAXwBNgEAVgF9AVEBBmcBfgFwAQZXAXkBXQEGfwFxAXMBBloBPwFyAQBiAYABIwEGbwGBAW0BBjcBggFDAQZn"
"AYMBhAEGZwGEAX4BBoUBWgFCAQkdAT4BdAEAYQGGAYcBCYABdwFrAQZjAYgBawEGSgGCATcBAG8BcQGJAQaKAUMBggEGPQE4AYsBAD4BjAF0AQAyAToBTAEJ"
"RgGFAUIBCXQBhgFhAQnAAGgAHgEGXQGNAY4BBowBjwF0AQBVAZABkQEGkgGTAVYBBpEBbAFVAQYyAUwBYAEJhQGUAVoBAGABhQFGAQmPAZUBdAEElgGGAXQB"
"BJUBlgF0AQRpAZcBSAEGeAGYAVIBBl8BXQGOAQZDAYoBJQEAJQGZATgBAD4BiwGMAQCHAZoBTAEATAFhAYcBCZgBmwFSAQaBAZwBiAEGnQGLATgBAD4BPQGL"
"AQCVAY8BjAENIgFFAWgBBkUBewFoAQZ9AXsBUQEGmwGQAVIBBpABVQFSAQY2AXwBSgEAjgGNAZ4BBmwBnwGDAQaFAWABlAEAyAAFAaABBnwBOwGhAQZvAYkB"
"gQEGfgGEAaIBBmcBbAGDAQaKAaMBJQEAjAGLAaQBAKQBlQGMAQB1AZsBRwEGRwGbAU0BBpsBpQFNAQahATsBaAEGawGIAYABBlYBpgF9AQZKAXwBoQEGeQGN"
"AV0BBokBnAGBAQaSAaIBkwEGkwGiAacBBn8BqAFxAQZEAakBegEGTAGqAWABAJoBhwGrAQBgAaoBlAEAhwGsAasBAJQBPwFaAQCVAa0BlgEArQGGAZYBDVYB"
"kwGmAQaRAZ8BbAEGSgGTAYIBAJkBJQGuAQBEAT8BrwEAhgGwAawBDSABYgEjAQZjAYEBiAEGfwGOAZ4BBqMBigGCAQaoAYkBcQEGrgGjAbEBBqMBrgElAQay"
"AasBpAEAhgGsAYcBDZcBdgFqAQZEAa8BqQEArwGzAakBALQBiwGdAQCLAbQBtQEAmgGrAbIBAJcBaQF2AQZpAbYBdgEGaQG3AbYBBrMBjQF5AQCeAagBfwEG"
"mQGdATgBALYBeQF2AQZKAaEBkwEAtwGzAbYBALYBswF5AQCEAacBogEGkwGjAYIBAJMBsQGjAQCTAacBsQEArgGnAZkBAJkBgwGdAQCkAYsBtQEAPwG4Aa8B"
"AK0BsAGGAQ2gAXUByAAGIwG5AR4BBpgBpQGbAQZ9AaYBaAEGugG7AbkBBroBvAG7AQCmAaEBaAEGvAG9AbsBAJABvgG/AQCxAacBrgEAsgHAAZoBALIBpAG1"
"AQCUAaoBwQEAuAE/AZQBAAUBHgGgAQZ1AaABmwEGaAF7AX0BBqABHgG+AQabAb4BkAEGuQHCAboBBsMBvgEeAQa+AcMBxAEAwwG7AcQBAJEBkAG/AQC6AcIB"
"vAEAnAHFAYgBBpMBoQGmAQC9AbwBxgEAvQHAAcQBAL0BxAG7AQC8AcEBxgEAxgGqAb0BAJoBwAG9AQCqAZoBvQEATAGaAaoBAMEBuAGUAQDBAaoBxgEAHgG5"
"AbsBBrsBwwEeAQagAb4BmwEGxAG/Ab4BAMQBwAG1AQCcAcIBxQEAnAHHAcIBAMcBwQG8AQC/AcQBtQEAxwGcAYkBAJ8BkQG/AQCfAb8BgwEAyAGoAZ4BAIkB"
"qAHIAQC/AbUBtAEAsgG1AcABAIMBvwGdAQDHAbgBwQEAtAGdAb8BAK8ByAGzAQCvAbgBxwEApAGrAZUBAK0BlQHJAQCAAYgBIwEGiAG5ASMBBrkBiAHCAQbF"
"AcIBiAEAwgHHAbwBAI0BswGeAQCzAcgBngEAxwGJAcgBAIQBmQGnAQDIAa8BxwEAhAGDAZkBAJUBqwHJAQCrAawBsAEAyQGrAbABAK0ByQGwAQDKAcsBzAEG"
"ygHMAc0BBsoBzQHOAQbKAc8B0AEGygHQAdEBBsoB0QHSAQbKAdIB0wEGygHTAdQBBsoB1AHVAQbKAdUB1gEGygHWAdcBBsoB1wHLAQbYAdkB2gEG2AHbAdkB"
"BtgB3AHbAQbYAd0B3gEG2AHfAd0BBtgB4AHfAQbYAeEB4AEG2AHiAeEBBtgB4wHiAQbYAeQB4wEG2AHlAeQBBtgB2gHlAQbmAecB6AEG5gHoAekBBuYB6QHq"
"AQbmAeoB6wEG5gHrAewBBuYB7AHtAQbmAe0B7gEG5gHuAe8BBuYB7wHwAQbmAfAB8QEG5gHyAfMBBuYB8wHnAQb0AfUB9gEG9AH3AfUBBvQB+AH3AQb0AfkB"
"+AEG9AH6AfkBBvQB+wH6AQb0AfwB+wEG9AH9AfwBBvQB/gH/AQb0AQAC/gEG9AEBAgACBvQB9gEBAgYCAgMCBAIABQIGAgcCAAMCCAIEAgAHAgYCCAIACAIJ"
"AgcCAAoCBQIHAgAHAgsCCgIAAwIMAggCAA0CDgIIAgAOAg8CCAIABwIJAhACABACCwIHAgARAhICEwIAFAIJAggCABQCCAIPAgAJAhQCFQIACQIVAhYCABcC"
"EAIJAgAKAhgCGQIAFwIJAhYCABYCGgIXAgAXAhoCGwIAFwIbAhwCAAoCHQIYAgAeAhQCDwIADwIfAh4CABcCHAIQAgAeAhUCFAIAFgIVAh4CACACHgIfAgAf"
"AiECIAIAGwIaAh4CAB4CIgIbAgAYAh0CIwIAGgIWAh4CACECJAIgAgAeAiUCIgIAIwImAhgCAB8CJwIhAgAhAigCJAIAJQIkAiICACQCKAIiAgAoAikCIgIA"
"KgIhAisCACoCKAIhAgAsAi0CLgIGLwIwAiwCBjECMgIzAgY0AjECMwIGMwI0AjUCBjYCNwI1AgY4AjkCNgIGOgIxAjsCBjwCDgI9AgYOAj4CPwIGPwI9Ag4C"
"BkACPwI+AgY+AkECQgIGQgI+AkACAEMCRAI4AgBEAkUCNgIANgJEAjgCBjQCNgJFAgBFAkYCMQIAMQI0AkUCADsCRgIxAgYxAjoCMgIGOgJHAj8CBkgCSQJK"
"AgZIAksCSgIGTAJLAkgCBk0CTgJLAgZPAkoCUAIGGwJRAlICBlICUwIbAgZTAlQCUgIAVAJVAlMCBlUCVgJUAgBVAlcCVgIAVwJYAlUCBlgCWQJXAgBKAksC"
"WgIGSgJbAloCAFsCTwJKAgZbAlwCTwIGUAJSAk8CBl0CTwJSAgZJAlACSgIGTgJeAksCBl8CQgJAAgY/AkACMgIGRwI9Aj8CAFcCVgJgAgZhAlcCWQIGSAJX"
"AmECBkkCVgJUAgZUAlICUAIGYgJjAmQCAGICZAJlAgBmAmcCaAIAaQJqAmsCBmsCbAJpAgZtAm4CbwIAcAJxAnICAG4CbQJzAgB0AnUCdgIGdwJ4AnkCBnoC"
"eQJ4AgZ4AncCewIGfAJ9An4CAH4COwJ8AgY7An8CfAIGAgJ5AnoCBnkCAgITAgYTAkQCeQIGegJ4An0CAH8CgAJ8AgB9AnwCgAIAOwJ+AoECBoICgwKEAgaF"
"AoYChwIGiAKJAocCBooCiwKMAgCNAooCjgIGgQKOAooCBo8CkAKRAgaSApMCjwIGjwKTApACBpMCkgKUAgaVApYClwIAXAKYApYCBpcClgKYAgaZAlwClgIG"
"kgKPApUCAJoCmQKWAgCWApUCmgIAUQJcApkCBpsCnAKdAgCeApgCnwIGoAKhAqICBqACogKjAgakAqUCpgIGpwKoAqkCBakCqgKnAgWrAqcCqgIFrAKrAqoC"
"BaoCqQKsAgWoAqwCqQIFqwKsAqgCBa0CrgKvAgWvArACrQIFsQKvAq4CBbICsQKuAgWuAq0CsgIFswKyAq0CBbACswKtAgWyArMCsAIFsQKyArACBbQCtQK2"
"AgW2ArcCtAIFtwK2ArgCBbgCuQK3AgW5ArgCugIFuQK6ArUCBbUCtAK5AgW1AroCuAIFuwK8Ar0CBb0CvgK7AgW+Ar0CvwIFvwLAAr4CBcACvwLBAgXAAsEC"
"vAIFvAK7AsACBbwCwQK/AgXCAsMCxAIAxALFAsYCAMYCxALCAgDHAsgCyQIAygLJAsgCAMgCywLKAgDKAscCyQIAzALNAs4CAM0CzALPAgDQAtEC0gIA0wLS"
"AtQCANEC1ALSAgDVAtYC1wIA2ALZAtoCANsC3ALdAgDeAt8C4AIA4QLiAuMCAN0C5ALVAgDeAuUC1QIA5gLnAugCAOkC3QLVAgDkAt0C3AIA5wLmAuoCAOkC"
"2wLdAgDVAuUC1gIA6wLsAu0CAO4C7QLiAgDiAuEC7gIA7wLwAvECANUC5ALeAgDkAt8C3gIA8ALyAvECAPMC9AL1AgD1AvYC8wIA9wLiAuoCAOoC+AL3AgD5"
"AvoC+wIA/ALZAtgCAP0C/gL/AgD/AgAD/QIAAQMCA9YCANYC5QIBAwADAwQDBQMABgMHA/8CAAYD/wIIAwDlAt4CCQMAAAP/AgcDAP8C/gIIAwAKAwsDDAMA"
"CwMDAw0DAA4DBAMDAwADAw8DDgMABgMBAwkDAAkDBwMGAwACAwEDBgMACQPgAgcDABADEQMSAwADAxADDwMAEwMUAxUDABUDFgMTAwAJAwED5QIACQPeAuAC"
"ABcDGAMFAwAEAw4DFwMAGQMXAw4DAAADGgP9AgAAAwcDGwMABwPgAhsDABwDGwPgAgAdAxwD4AIAHgMdA+ACAOAC3wIeAwAfAx4D3wIAIAMfA98CAN8C5AIg"
"AwDcAiAD5AIAGQMAAxsDBhsDHAMdAwYZAxsDHQMGGQMdAx4DBvYCGQMeAwYeAx8DIAMG9gIeAyADBtsC9gIgAwbbAiAD3AIGIQMiAyMDByQDIQMjAwElAyYD"
"JwMAKAMnAyYDBikDJQMqAwAqAygDKQMGJQMqAycDBicDKAMqAwArAywDLQMGKwMuAywDAC4DKwMvAwAsAzADLQMGLQMuAy8DBi4DLwMwAwcwAywDLwMHMQMy"
"AzMDBjQDNQM2AwA3AzgDNAMANAM5AzcDADQDeQI6AwA0AzoDNQMANAM4AzsDADsDOAM8AwA9Az4DOwMAPwM8A0ADAD0DQANBAwA+AzQDOwMAQgNBA0ADAD4D"
"PQNBAwA0A0YCeQIANAM+A0YCAEMDQAM8AwA8A0QDQwMAQQNFAz4DAEADQwNCAwB7AkYCPgMARQNGAz4DAEUDQQNHAwBIA0kDRQMARQNHA0gDAEUDSQNGAwBK"
"A0YDSQMARgNKA0sDAEgDTANKAwBNA0oDTgMATgNKA0wDAEoDTQNLAwBNA04DTwMAUANRA04DAFIDTwNOAwBNA08DUwMAUgNOA1EDAFEDVANSAwBSA1QDVQMA"
"UwNPA1UDAFYDUwNXAwBTA1UDVwMAWANNA1MDAFEDWQNUAwBZA1oDVAMAUwNWA1gDAFcDWwNWAwBXA1UDXAMAXQNcA1UDAD0DOwM/AwBeA18DYAMGXwNeA2ED"
"BmIDYwNeAwZeA2MDYQMGYQNkA2UDBmQDYQNjAwZmA2EDZQMGZQNnA2YDBmcDaANpAwZpA2oDZwMGawNmA2cDBmwDbQNuAwZsA2cDagMGagNtA2wDBm4DbwNs"
"AwZrA2cDbAMGbANwA2sDBmwDcQNwAwZxA2wDcgMGcgNzA3EDBnQDcwNyAwZ0A3UDcwMGcwN1A3YDBnMDdgN3AwZ2A3UDeAMGeQN3A3YDBnoDaAN7AwZ7A3wD"
"egMGaAN6A30DBnoDfAN+AwZpA2gDfQMGfgN9A3oDBn8DbwNuAwaAA4EDfwMGbwN/A4EDBm8DgQOCAwZlA2QDewMFewNoA2UDBWUDaANnAwWDA4QDagMFagOE"
"A4UDBW0DagOFAwVyA2wDbwMFcgNvA4IDBYIDdANyAwU4AzcDRAMGRAM8AzgDBlUDVANaAwZaA10DVQMGNgMRAhMCAIYDhwOIAwCHA4YDiQMALgKKA4sDBowD"
"jQMvAgaMAywCiwMGiwMsAi4CBowDLwIsAgZCAjgCjgMGOQI4AkICBo4DQgJBAgZYAo8DWQIAkANYAo8DBpEDkgOTAwaUA5UDlgMGkQKQAl4CBpcDmAOZAwY5"
"AzQDNgMAXAOIA1cDAJoDYgNeAwaaA14DYAMGdgN4A5sDBnYDmwN5AwacA50DngMGnAOfA50DBp8DoAOdAwafA6EDoAMGogOjA6QDBqMDpQOkAwajA6UDpgMG"
"pgOlA6cDBqYDpwOoAwaoA6kDpwMGqAOqA6kDBqoDqwOpAwaqA6wDqwMGrQOuA68DBq0DsAOuAwaxA7IDswMGswOyA7QDBrMDtAO1Awa2A7cDuAMGtwO4A7kD"
"BrcDuQO6Awa6A7kDuwMGugO8A7sDBrwDuwO9Awa8A74DvQMGvAO/A74DBr8DvgPAAwbBA8IDwwMGwQPDA8QDBsQDwwPFAwbEA8UDxgMGxwPIA8kDCsoDywPM"
"AwrNA84DzwMKPobhPO1fQ5QBTpNShonaPMNGfohQDCc8qIh1FPs8cYjxGOzOJoZoDd/LAYjpLgTNh4hJRBO7Pp93XEmb3YiRMpypGJNmQENrHpg8FrlvB4/y"
"Gd1p7pwgEDxiHYx+HpGar4tpG5SkG5o7V5oWUZdWWN8gj5U0EMOagrQ6Ei2rR5mrOJVwxKZQOriyaafuWdMqYaeHFWzKiqfxMKbLKZUQMHlA3aR4PMy9GXYH"
"DDkgI7U4DMlFUKbkFRFCRZ1SW//hPrF8LU1i1qx3Kiyq4KP1O6tVNa0yQ2RydrejacmvP5IWX8Aa+KgvaZsVyrQIDgFf1bcVEoF5jKajY2fAULCzcnPWm8aD"
"Wgs2Mrt0dpANXbNVeGf6iLwbJ8157bffN6R3Xb5IYmqhC72pexTcgL1kfyoCddR2PCV9EQqrUwBo5b8Sc/XAbgciEKWe8dt2Yjeed/hHEQ1+4tkxHMZ9if+i"
"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"
"ADwAPQAGSwA9AEwABjwATQA0AAArAEoARAANRwAyAD4ADUsATQA8AABJAEMAOAAAQwBMAD0ABkAATgAGAAY3AEYAQgAASABPAEYACUYATwBCAABQAEwAQwAE"
"OgBFAEgADVAAQwBJAARRAE0ASwAASwBMAFEAAFIABgBOAAZHAEQAUwAJBgBUABwABk0AVQA0AABHAEUAMgANQAA2AFYABlcASQAdAAZBAFYANgAGTABQAFEA"
"AB0AHABYAAZYAFcAHQAGRQBHAFkAAE4AQABWAAZaAEkAVwAGSgBTAEQADUgAWwBPAABIAFwAWwAAUwBZAEcAAEgAWQBcAABIAEUAWQAAUABdAFEAAAYAUgBU"
"AAZJAF4AUAAAWQBfAGAACUkAWgBeAAY0AFUASgAAQgBPAFsAAF8AWQBTAABhAGIAYwAATQBRAFUABlUAUwBKAABkAGUAXAAJWwBcAGUAAGYAWgBXAAZnAGgA"
"aQAGZABbAGUAAFEAXQBVAAZjAGoAYQAGawBsAFIABm0AVQBdAAZZAGQAXAAJYgBuAGoABmoAYwBiAAZZAGAAZAAJQQBnAFYABm8AbgBwAAZuAGIAcAAGUABe"
"AF0AAFgAHABvAAZxAHIAawAGXwBTAHMAAGgAdABpAAZyAGwAawAGZgBhAHUABnYAcgBxAAZgAHMAZAAAXwBzAGAAAGYAdQBaAAZ2AGkAdAAGdwBTAFUAAEEA"
"aABnAAZbAHQAaAAAdAByAHYABl0AXgB1AABBAFsAaAAAagB1AGEABnUAbQBdAAB1AFUAbQAAcwBTAHcAAGwAbgBUAAZsAHgAbgAAZABzAHgAAG8AHABuAAZq"
"AG4AdwAAdQBeAFoAAHgAdwBuAABzAHcAeAAAeABbAGQAABwAVABuAAZyAHgAbAAAagB3AFUAAHIAWwB4AABSAGwAVAAGWwByAHQAAGoAVQB1AAB5AHoAewAG"
"eQB7AHwABn0AfgB/AAZ9AIAAfgAGgQCCAIMABoEAgwCEAAaBAIQAhQAGhgCHAIgABoYAiQCHAAaGAIgAigAGiwCMAI0AAIwAjgCNAACPAJAAjgAAkQCSAI4A"
"AJMAjwCOAACUAI4AkgAAjwCVAJYAAI4AlACXAACOAJcAkwAAkgCYAJQAAJIAmQCYAACYAJoAlAAAlACaAJcAAJoAmwCXAACcAJ0AngAGnwCgAKEABqIAowCk"
"AAalAKQAowAGiwCeAKYABqcAqACpAAaXAKUApwAGqgCrAKwAAK0ArgCvAACwALEAsgAAswCsALEAALQAtQC2AACsALIAsQAAtwC1ALQAALIArACrAAC3ALIA"
"qwAGqgC3AKsABrgAuQC6AAa4ALsAuQAAuwC4ALwAALkAvQC6AAa6ALsAvAAGuwC8AL0AB70AuQC8AAe+AL8AwAAGwQCeAMIAAMEAwwDEAADFAMYAwQAAxwDB"
"AMYAAMEAoACeAADBAMcAoAAAwQDEAMUAAKYAoADHAADGAMgAxwAAxwDIAMkAAMkAyADKAADJAMoAywAAyQDLAMwAAMoAzQDLAADMAMsAzgAAzQDOAMsAAM8A"
"0ADRAAbRANAA0gAG0wDQANQABtUA0wDUAAbUANYA1QAG1gDXANUABtIA0wDVAAbVANgA0gAG1wDZANgABtkA1wDaAAbbANQA0AAG1wDWANwABtIA0ADTAAXd"
"AN4A1AAF1ADeANYABdgA1QDXAAXCAN8AiwAA4ADhAOIABuAA4wDhAAapAKgA5AAGwwDBAMIAAM4A5QDMAAA="
;
static const char* CARDATA_OLDSEDAN =
"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"
"AIgAAJwAVgA2AAd8AIoAKwAAGwA6AGkAAI0AnQB9AACSAGsAkQAEbQCeAGwAAJ4AbQBvAAWPAGYAcgAJggCXAEIAAYMAdgCfAAmYAKAAWAAAkAChAEQAAIkA"
"SwAqAAA6AKIAowAApACLAFkAAE8AkgBdAASlAC8ApgAAZgCPAKcACZcAggB1AAl2AI4AnwAJqACOAIEAAYEAqQCoAAmqAEoAhAAHSwCJAIgAB1kAjACrAABo"
"AFkAqwAAaACQADkAAKEATABEAAcPAJoAmQAHmwCaAEsAB5sASwCIAAdWAKwAVwAGnACsAFYABn0ApABZAACUAGAAXgAApgAvAGAAAKUAlgAvAACVAJMAfgAA"
"rQCAAEYAAJYAfwBDAACDAK4AZwAAgwCfAK4AAI4ArwCfAAlmAKcAqQAJsACPALEAAbEAjwCXAAGxAJcAsgABswANACkABw8ADQCzAAdYAKIAOgAAoACiAFgA"
"AGgAtACQAACQALQAoQAHhQC1AIYAB6QAfQCdAABpADoAowAAaQCjALYAALcAbwCTAAB/ALgAdAAArwCOAKgAAZ8AuQC6AACPALAApwAJuQCfAK8AAKoAeQBK"
"AAcPAEsAmgAHuwCYAIoAB6sAtABoAAe8AIYAtQAHegCcADYAB2kAvQBqAAe9AJEAagAAbAC+AI0AB28AtwC/AADAAK0ARgAARgBnAMAAAGcAwQDAAAC5AK8A"
"qAABKQCGALMAB70AaQC2AAeeAL4AbAAAngDCAL4AAMIAngC/AAWeAG8AvwAFwQCAAK0AAcEArQDAAAHDAMEAZwAArgDEAGcAALIAlwDFAAmuAJ8AugAAgQBm"
"AKkACaoAhABLAAeMALQAqwAHTADGAMcABlUATADHAAbIAJIAkQAEXQCSAMkABMoAgADBAAGWALgAfwAAwwDKAMEAAMsAdQB0AAC5AKgAqQAAxQCxALIAAbMA"
"SwAPAAeYAMwAoAAHjADNALQAB6EAxgBMAAeTAM4AtwAAzwDKAMMAAa4AugDQAACXAHUAxQAJvADRAIYAB7MA0QBLAAegAMwAogAHzQChALQAB9IAuwCKAAei"
"AMwA0wAAjQDUAJ0AANQAjQC+AAeSAMgAyQAE1QClAKYAAZYApQDVAAm5AKkA1gAAsQDFANcAAIYA0QCzAAeYANgAzAAHowCiANMAANQA2QCdAAC9AMgAkQAA"
"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"
"AWABVgEBPwFhASYBCWIB5wALAQdiAfsA5wAHTAFIAVoBAFcBYwE7AQBjAU8BOwEAqgAyAd8ABwYBZAHeAAdkAaoA3gAHLQFJAV0BAGABQgFWAQkqAUsBUgEA"
"SQFlAV0BAFUBRQFQAQAyAVgBCgEHHQHNAB4BB0MBzQAdAQcVAWYBMwEHYwFXAWcBB08BFwE5AQBoAWkBVQEAYAFqAUIBCUcBUgFrAQBjAWwBTwEAUAFoAVUB"
"AEcBaAFQAQBCAVoBSAEJbQFfAQIBAFUBbgFZAQBgAVEBbwEBbwFqAWABAWYBQwEzAQcCAT0BbQEATQFmARUBBxoBXQFNAQdwAWUBSQEHXwFnAVcBB0sBJgFh"
"AQkZAUQB1gAATQFdAWYBB2UBcAFdAQBZAXABSQEHWQFxAXABB3ABcgFdAQBfAW0BcwEAVQFpAW4BAEcBawFoAQBCAWoBdAEJSwF1AVIBAHMBdgFfAQBnAV8B"
"dgEHTwF3ARcBAFEBWwFvAQBaAXgBMQEAeQFEARkBABkBMQF6AQlnAWwBYwEHbwF0AWoBAXUBSwFhAQkZAXsBeQEJXQF8AWYBB1sBfQFvAQB+AT8BUwEBfwF+"
"AVMBAVgBYgELAQeAAWcBdgEHdwGBARcBAE8BbAF3AQAPARcBWwEAfgFhAT8BCVMBRAF/AQBEAYIBfwEAewGDAXkBCQ0BZAEGAQf7AGIBXAEHcgF8AV0BB3YB"
"cwGAAQCEAXUBYQEJegF7ARkBCYUBMgGqAAdZAW4BcQEAMQGGAXoBCXkBgwFEAQBkAYUBqgAHhwGAAXMBAIABhwGIAQBnAYABiAEAFwGBAYkBAHgBWgGKAQEx"
"AXgBiwEAbQGMAXMBAI0BdwFsAQB9AXQBbwEBjgFCAXQBCY4BjwFCAQkxAYsBhgEAZgHNAEMBB3MBjAGQAQdnAYgBjQEAbAFnAY0BAGkBaAGRAQCSAZEBawEA"
"UgGSAWsBAIQBYQGTAQmUAQ0B0QAHFAGUAdEAB5UBXAFiAQeRAZYBaQEAYQGXAZMBCX4BlwFhAQlzAZgBhwEAFwGJAVsBAIsBeAGKAQF/AYIBmQEAXAGaAT0B"
"AJABmAFzAQdwAXEBmwEHfQGOAXQBAZwBnQGeAQmZAZwBfgEBnwFYATIBB24BoAFxAQCIAYcBoQEEjQGIAaEBBFoBQgGiAQl+AX8BmQEAgwGCAUQBAHsBowGD"
"AQBUAVwBPQEAfAFyAaQBB2kBlgFuAQWlAZIBUgEAdQGlAVIBAIoBWgGiAQmGAYsBpgEAXgGUARQBB2IBWAGfAQdyAXABpwEAkAGoAZgBB3ABmwGnAQBoAWsB"
"kQEAdQGEAaUBCZMBqQGEAQmUAWQBDQEHlQGaAVwBB3wBqgFmAQeQAYwBqwEAmAGoAawBAJgBrAGHAQR+AZwBngEJrQGLAYoBAa4B8gDNAAevAV4B8gAHrAGh"
"AYcBBG4BlgGgAQWiAa0BigEJggGcAZkBAIMBowGCAQB6AbABewEAsQEyAYUBB7EBnwEyAQeqAc0AZgEHrwGyAV4BB58BswFiAQdtAasBjAEAcQG0AZsBB6UB"
"hAG1AQGpAbYBhAEJlwG3AZMBCYIBuAG5AQCwAaMBewEArgGvAfIAB68BugGyAQayAbsBXgEHXgG8AZQBB70BvgGfAQe+Ab8BnwEHnwG/AbMBB7MBlQFiAQeo"
"AcABrAEAcQGgAbQBAKwBwAGhAQCNAaEBwQEAfQHCAY4BAZcBfgGeAQmCAZ0BnAEAlAHDAWQBB8QBvQGfAQdtAT0BxQEAcgGnAaQBAKsBqAGQAQfAAcEBoQEA"
"tgG1AYQBAcYBngGdAQGLAa0BxwEAegGGAaYBAHoBpgHIAQBkAcMBhQEHyQHKAZQBB68BywG6AQbMAZUBswEHmgHNAcUBAI0BwQF3AQDOAakBkwEJuQGdAYIB"
"AMoBwwGUAQe8AckBlAEHzwHEAZ8BB8EBgQF3AQCOAdABjwEJxgGXAZ4BAJ0B0QHGAQDIAbABegEAlQHSAZoBB6QB0wF8AQCrAdQBqAEHjgHCAdABAY8BogFC"
"AQnRAZcBxgEApgGLAccBALAB1QGjAQl8AdMBqgEA1gHJAbwBBz0BmgHFAQCbAbQBpwEAwQHAAdcBBJEB2AGWAQDCAdkB0AEBfQHaAcIBAJMB0QHOAQHRAZMB"
"twEBlwHRAbcBALgBowG5AQmCAaMBuAEA2wGwAcgBAW0BxQGrAQDBAdcBgQEE2AGgAZYBBdwBiQGBAQBbAYkB3AEAWwHaAX0BAN0BtQG2AQHeAakBzgEB0QHe"
"Ac4BAd8BxwGiAQCiAccBrQEAuQHgAZ0BANUBsAHbAQHJAeEBygEHwwHiAYUBB+MB5AHNAAfkAa4BzQAH5QGVAcwBB6gB5gHAAQDnAbQB6AEA6AG0AaABAOgB"
"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"
"AmACBlkCYAJhAgZZAmECYgIGWQJjAmQCBlkCZAJaAgZlAmYCZwIGZQJnAmgCBmUCaAJpAgZlAmkCagIGZQJqAmsCBmUCawJsAgZlAmwCbQIGZQJtAm4CBmUC"
"bgJvAgZlAm8CcAIGcQJyAnMCBnECdAJyAgZxAnUCdgIGcQJ3AnUCBnECeAJ3AgZxAnkCeAIGcQJ6AnkCBnECewJ6AgZxAnwCewIGcQJzAnwCBn0CfgJ/AgZ9"
"AoACfgIGfQKBAoACBn0CggKBAgZ9AoMCggIGfQKEAoMCBn0ChQKEAgZ9AoYChQIGfQKHAogCBn0CfwKHAgaJAooCiwIGigKMAosCBooCjQKMAgaNAo4CjAIG"
"jQKPAo4CBo8CkAKOAgaRApICkwIGkQKSApQCBpQClQKSAgaWApcCmAIGlgKZApgCBpkCmAKaAgabApwCnQIGnQKeApwCBp0CnwKeAgafAqACngIGoQKiAqMC"
"BqMCpAKiAgajAqUCpAIGpQKmAqQCBqUCpgKnAganAqgCpgIGpwKpAqgCBqoCqwKsAgaqAq0CqwIGrQKhAqsCBq4CrwKwAgavArECsAIGsgKzArQCBrUCtgK3"
"Aga1ArcCuAIGuAK3ArkCBrgCugK5Aga6ArkCuwIGvAK9Ar4CBrwCvwK9Aga/AsACvQIGvwLBAsACBsECwgLAAgbBAsMCwgIGwQLDAsQCBsECxQLEAgbFAsYC"
"xAIGxQLHAsYCBsUCxwLIAgbIAscCyQIGyALKAskCBsoCywLJAgbKAswCywIGygLNAswCBrwCwQK/Aga8AsUCwQIGvALIAsUCBrwCygLIAga8As0CygIGzgLP"
"AtACBs4C0ALRAgbRAtAC0gIG0QLSAtMCBtMC0gLUAgbTAtQC1QIG0wLWAtUCBtYC1wLVAgbWAtcC2AIG2ALXAtkCBtgC2QLaAgbYAtsC2gIG2ALbAtwCBtwC"
"2wLdAgbcAt0C3gIG3ALfAt4CBs4C0QLTAgbOAtMC1gIGzgLWAtgCBs4C2ALcAgbOAtwC3wIG4ALhAuICBuAC4QLjAgbjAuEC5AIG4wLlAuQCBuMC5QLmAgbm"
"AuUC5wIG5gLoAucCBuYC6QLoAgbpAuoC6AIG6QLrAuoCBukC7ALrAgbsAu0C6wIG7ALtAu4CBuwC7wLuAgbvAvAC7gIG7wLxAvACBu8C8gLxAgbgAuYC4wIG"
"4ALpAuYCBuAC7ALpAgbgAu8C7AIG4ALyAu8CBvMC9AL1AgbzAvUC9gIG8wL3AvYCBvMC9wL4Agb4AvcC+QIG+AL5AvoCBvgC+gL7Agb4AvsC/AIG/AL7Av0C"
"BvwC/QL+Agb8Av8C/gIG/wIAA/4CBv8CAAMBAwYBAwADAgMGAQMCAwMDBgEDBAMDAwYBAwQDBQMGBQMEAwYDBgUDBgMHAwbzAvgC/AIG8wL8Av8CBvMC/wIB"
"AwbzAgEDBQMG8wIFAwcDBggDCQMKAwALAwoDCQMACQMMAwsDAA0DDgMPAwENAw8DEAMBDwMOAxEDBxADDwMRAwcNAxADCQMHCAMJAxADBxEDCAMQAwAOAw0D"
"EgMAEwMUAxUDABMDFQMIAwAIAxEDEwMAFAMTAxYDAA4DFgMTAwASAxYDDgMADgMTAxEDABcDGAMZAwAZAxoDFwMAGwMcAxgDBxgDHQMbAwceAxsDHQMAHAMf"
"AyADAB0DIQMiAwAiAx4DHQMAIwMiAyEDACMDHwMiAwAjAyADHwMAHgMiAx8DACQDHwMcAwEcAxsDJAMBHwMkAx4DByQDGwMeAwElAyYDJwMPnpS2VEglBZG4"
"GRQYjIxcJ2xA2olTPwtQpI5XEOrPCaAND8/MApGlIuTqz6HqD+UdbJWVE+VDJYtzJ3rRAYwwRin99Y6tSqDNLovTOFZg14yeJqZo0pcwLjB6547XTNX/hpau"
"VK8QbqEIV58ze4rSOdPH7YmCPzW6A5YNVRnyqpYmVzPWC5wsDpJk4ZgONyaFh5EcE1ioS4xpML2rpJQrRH+PCbLoDoh4Tpc3WVnGZaDYDCw/opZaOUp2cZaS"
"VFD9+qnhRypyhanIYBarPKqGEceDEqU9EGmWB6zjOsqDO5w2TCuGNKUja9sY4KtwdQMKCqodcwfWQ6BGZtDJIa5td1fyO6FtEP6t1rOhNLJ5pL1YXC+eWrBe"
"Ur5UxLajVYWEQ7iXdgTEC6z8doD4mbVXT5+DFreNDTZGNMZYMfZ7IL87Ny6Cqrphdy0W/LXgDsXKBE0lEA6E7MgDfYMKLcszfxjwBrcyEFquWraNDi1iAsFp"
"Qdh2uc1aELiFjrN+D2a7bdTsL/19u83SVs07l1e+GgZ/0Q89ENOCktbNRGKCgVuPSXtwWATiVqA/uNABQ/N4EukKeYPB2gD4V0+EqQGpNm+BvQhpXG+a/BEv"
"WLY6WQsGeLQa9DVjfV0Kgf/VRWd5XSpXIfWAaDb/f+Pv/CLneMzBm0+zDyR4qkjmDF1FWTWoMM58UkoaD4pi9jaOS7OCBlaEWL43aS8ZQ9F4T2AEN5CDRzE5"
"PjGBY0IXXJee+krvDvbKZkfnd4EVokjRPNp3mVpDT+eBS1MraFC0dmLFDrxlYkhldvHEUFtRWBwyWVOEcw4QeEhVENmublA/eObyrFboW02m0V6eD1RAplGU"
"d0T4clQdcx3VYF/hDmfMQGciMDF5eWLxXIQjbl1TEPSt7msKQQ6PXXHbFKs/lG3cD0+p6WDFFQeG1WusEZlFzGfJVw/DdWgqUrUNcGjoUwDzamZ2VnP8umfl"
"VTzW9GjVVP3ntnOPP35XnWkLVBUiiHOqNZVFr26kShfN/nJTLUZn5nB7SFoC/W9+I9mounETE2/RVnIFKCrRyHPJOf2wZnS0PR3E+G2SHHhH+G2yMIJN+G0w"
"KMhg+G3cDBpe+G1xCIJNYm6yMELEYm6SHEzKYm5xCELEYm7cDKuzYm4wKP2wC5KSHKlhC5LcDAhLC5JxCJ9bC5IwKFpIC5KyMJ9bpJHwEPywpJHwEGzJpJGv"
"MELEpJH8BjS9pJFELKqzU1CTHFxpe1C9QDBb1nEbHUZq2HErQPFX9olxPpBUjnPtPf5QmHNmPIdNi4nnOUhKU1DdOCZJ/HMxNwVHTVCjHDE+LXCcHMA/tq+T"
"HFtpTY0sHedqta82PxBdk4v7QIVUl3TtPf5Qv4p9PINNtq+yN61HdYqENllHG3RXMtBEAYuPLbZCtq/CJq1Au4xzIlFABHGSHAFAtq+SHAFAK1FeHbbSz4tz"
"InTRJnInKIfQSoqBLQLPaHNNM6bNJVCmQMS7G3NTPeq1QornOe6yAHS4NuGvMFCuHGqmyW2SHKWotq+YHK3SgItRINHNMHMnKIfQb4mHLfXO0a9RPQzHPorK"
"O6fA8HRxPjW9k4oFPqG5tq/6OmGzTIvnOe6yXXOENv2v0ItnMmutvK+iHMqmp5C2HO2n7GYkXLokV3UHWIMiGGtuUfwhyJhtXHckQZW1UYMkpIpTWGEiAAAB"
"AAIAAAAAAgADAAAEAAUABgAAAgAHAAgAAAQABgAJAAAGAAEACgAACQAGAAsAAAwADQAOAAAKAA8ABgAAAQAQAAoAABEAAAADAAAAABAAAQAAEgALABMAAAkA"
"CwASAAAUABUACwAABgAPABQAAAsABgAUAAABAAcAAgAADQAWAA4AABcAGAAZAAAaABkAEwAADgAWABsAABkAGgAXAAAPAAoAEAAAEwALABwAAAgABwAdAAAM"
"AA4AHgAAHwAUAA8AAA8AEAAfAAAaABMAHAAAHgAgAAwAACAAAwAMAAAVABwACwAAIQAaABwAACIAIwAYAAAXABoAJAAAJQAkABoABSAAEQADAAAQACYAJwAJ"
"FwAiABgAAB8AEAAnAAkQAAAAJgAJIQAlABoAABUAFAAoAAkcABUAKQAJHAApACEAAAEABgAFAAAqABQAHwABIwArABgAAA4ALAAeAAAhAC0AJQAABQAHAAEA"
"AB4ALAAgAAQVACgAKQAJLgARACAAAC0ALwAlAAApADAAIQAAFAAqACgACTEAKgAfAAEyACUALwAAAAARACYAAB0AMwAIAAc0ACwADgAAJAA1ABcAACQAJQAy"
"AAUwACkAKAAANgAnACYAAB8AJwAxAAkFADcABwAHOAArACMABxcANQAiAAAnADYAOQAAMQA5ADoAADgAOwArAAc2ACYAEQAAKAAqADoAACoAMQA6AAA8ABsA"
"FgAHPQAgACwAADEAJwA5AAA3AB0ABwAHIgA+ACMABz4AOAAjAAc3AD8AOwAHNAA9ACwABDgANwA7AAcbAEAADgAAEQAuAEEAAEAAGwBCAABAADQADgAHIgBD"
"AD4AADIARAAkAAUuAEUARgAAMAAtACEACUcAPQA0AABHACAAPQAAOgBIACgAAEAARwA0AAAtAEkALwAANQBKACIAAC4ARgBBAABEAEoAJAAAMABIAEsACSgA"
"SAAwAAAtAEsASQAATAARAEEACTIALwBJAAA1ACQASgAALQAwAEsACREATABNAAk2ABEATQAJOgA5AE4ACU8ARwBAAAdJAEoARAAARwBPACAAADIASQBEAAAi"
"AEoAUAAANgBNAE4AAE8ALgAgAAA2AE4AOQAAOgBRAEgAAFIASwBIAAkbAFMAQgAARgBMAEEAASIAUABDAABIAFEAUgAALgBPAEUAAFQAPAAzAAdCAFUAQAAA"
"VQBPAEAAB1YAGwA8AAcdAFQAMwAHNwBUAB0AB0MAOAA+AAdJAFcASgAATABGAFgAAVUAWQBPAAdKAFoAUAAASgBXAFsAAEsAXABJAABQADgAQwAHVABWADwA"
"B08AWQBFAABGAEUAWAAAXQBUADcABxsAVgBTAAdOAFEAOgAJSgBbAFoAAE4ATQBeAABfAEUAWQAAWQBVAF8AAFcASQBgAABcAEsAYQAJYgBTAFYAB0sAYwBh"
"AAlSAGMASwAJUgBRAGMAAFgAZABMAAFNAEwAXgAJXABgAEkAAE4AXgBlAABmADcAOAAHZgBdADcAB04AZwBRAABhAGgAXAAJXQBpAFQAB1AAWgA4AABOAGUA"
"agAAawBjAFEAAGwAaQBdAAdqAGcATgAAZABeAEwACVEAZwBrAABfAG0ARQAEYABbAFcABW4AZQBeAABvAGYAOAAHVQBtAF8AAHAAWgBbAABwAFsAYAAFaQBx"
"AFQAB28AOAByAABaAHMAOAAAYQBwAGgAAHEAdABUAAdgAFwAaAAAYQBjAGsAAG4AXgBkAABrAHUAYQAAZQBuAHYACXcAZwBqAAE4AHMAcgAAdgBqAGUACXcA"
"agB4AAFCAFMAYgAAaABwAGAAAHYAeABqAAl5AGsAegAJawB5AHUACW0AewBFAABYAEUAewAAbgB8AHYACWsAZwB6AAl3AHoAZwAJVQBCAG0AAHAAYQB1AAB2"
"AHcAeAAHZAB9AG4AAHkAfgB1AABCAGIAfwAAWAB9AGQAAHYAgAB3AABCAH8AbQAAWgByAHMAAHsAfQBYAAB3AIAAfgAAdQB+AHAAAIEAcgBaAABtAH8AewAA"
"fQB8AG4AAHwAgAB2AAB5AHoAfgAAbACCAGkAAGkAggBxAACCAIAAcQAAgACCAIMAAFoAcACBAACBAHAAhAAAfQCAAHwAAHAAhQCEAACFAHAAfgAAegB3AH4A"
"AIAAfQBxAAB+AIMAhQAAfgCAAIMAAIYAhwCIAAaGAIgAiQAGhgCJAIoABosAjACNAAaLAI0AjgAGiwCOAI8ABpAAkQCSAAaQAJMAkQAGkACUAJMABpUAlgCX"
"AAaVAJgAlgAGlQCXAJkABpoAmwCcAAabAJ0AnAAGmwCeAJ0ABpsAngCfAAabAKAAnwAGmwChAKAABpsAoQCiAAaiAKEAowAGogCkAKMABqQApQCjAAaaAKIA"
"mwAGmgCkAKIABqYApwCoAAaoAKcAqQAGqACpAKoABqgAqwCqAAaoAKsArAAGrACrAK0ABqwArQCuAAasAK8ArgAGrACvALAABrAArwCxAAawALEAsgAGsACz"
"ALIABqYAqACsAAamAKwAsAAGpgCwALMABrQAtQC2AAa0ALcAtgAGtAC3ALgABrQAuQC4AAa5ALoAuAAGuQC7ALoABrkAuwC8AAa5AL0AvAAGvQC+ALwABrQA"
"vQC5AAa/AMAAwQAGvwDCAMEABr8AwgDDAAbDAMIAxAAGwwDEAMUABsMAxgDFAAbDAMYAxwAGxwDGAMgABscAyADJAAbHAMoAyQAGxwDKAMsABssAygDMAAa/"
"AMMAxwAGvwDHAMsABs0AzgDPAAHQANEA0gAB"
;
static const char* CARDATA_SPORT1 =
"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"
"ACUAJAAARgBEACYABj4ASQATAAAcADcASgAAAQBAAB8AAAMAQAAGAAAeACUASwAARwAjABoAADQAPQAtAAAbAEYAQwAAAgAwADYAACcAPwBMAAZMAD8AOQAG"
"DwAiAE0AAEEATgAiAAAlAE8ASwAAQQAeAFAAATYARgAbAAAFABMAUQAAMgBSAAMAAFIAQAADAAAqABkAUwAAVAA7ADwAAFQARwA7AABVACIATgAAVgBQAB4A"
"CU8AHgBLAAklAEIATwAAUQAyAAUAAEgADwBNAABCACMARwAALQA9AFcAADkAWABMAAYQADoAMwAAUgBZAEAABh4ATwBaAAlaAE8AQgAAVQBOAFsACUYAXABE"
"AAZcAF0ARAAGMgBRAF4ABl8AQABZAAZgACIAVQAASQBRABMAADAAYQA2AAYwAF8AYQAGXwAwAB8ABl4AYgAyAAYyAGIAUgAGHwBAAF8ABlgAOQAzAABjADwA"
"KgAAZABCAEcAAFYAHgBaAAlgAFUAWwAJWwBOAEEACUEAUABWAAkvAGUAPQAGZgBRAEkABmEARgA2AAZeAFEAZgAGDgBnADgABjgAZwA6AAZiAFkAUgAGYwAq"
"AFMAAEgAaAApAABGAGEAXAAGNwBEAF0ABl0ASgA3AAZFAGUALwAGRQAnAEwABg4AHABpAAY6AGoAMwAGawBIAE0AAE0AIgBgAABWAGwAQQAJZQBXAD0ABl0A"
"HABKAAZpAGcADgAGWAAzAGoABkcAVABtAAFBAG4AWwAJWgBCAFYAAG8AYABbAABCAGQAcAAAbwBNAGAAAG4AQQBsAAk+AHEASQAAaQAcAF0ABmoATABYAAZq"
"ADoAZwAGXwByAGEABmYAcwBeAAZiAF4AWQAGbwBrAE0AAFYAQgBwAAB0AHUAdgAGSQBxAGYABl8AdAB2AAZfAHYAcgAGWQBeAHMABlkAcwB0AAZZAHQAXwAG"
"dwBIAGsAAG4AbwBbAABkAEcAbQABdQB4AHYABnYAeQByAAZ6AGUAewAGRQB7AGUABkUATAB7AAZvAHwAawAAbABvAG4AAHgAeQB2AAZ9AH4AfwAGewB/AIAA"
"BnsAfQB/AAZ6AHsAgAAGewBdAH0ABl0AewBnAAZpAF0AZwAGagB7AEwABmoAZwB7AAaBAG0AVAAAdwCCAEgAAEgAggBoAACDAHwAbwAAcwCEAHQABoUAVAA8"
"AACGAGsAfAAAhwBcAGEABnIAiABhAAaBAFQAhQAAiQCCAHcAAIgAhwBhAAaKAFYAcAAAbACLAG8AAIoAbABWAACMAHMAZgAGdACEAHUABowAhABzAAaFADwA"
"YwAAjQBkAG0AAYsAgwBvAACOAIsAbAAAcQCPAGYABo8AjABmAAZ3AGsAhgAAkACKAHAAAIgAcgB5AAaBAI0AbQAAkQB/AH4ABnkAeACIAAZ1AIQAeAAGeACE"
"AIgABnwAkgCGAAGOAGwAigAAkQCTAH8ABnAAZACNAAGSAIkAdwAAkgB3AIYAAIsAfACDAAGUAIgAhAAGlACHAIgABpUAfgB9AAaWAHoAgAAGlwBXAGUABpgA"
"fACLAAGQAJkAigAAhACaAJQABpQAmgCbAAaLAJwAmAAAnACLAI4AAFwAnQBdAAaUAJ4AhwAGnwB8AJgAAY8AoACMAAaHAJ4AXAAGoQBlAHoABpoAogCjAAaa"
"AKMAmwAGjQCQAHAAAZsAowCkAAalAIQAjAAGnwCSAHwAAaYAngCUAAafAKcAkgABqACfAJgAAV0AqQB9AAaqAIUAYwAAlQCRAH4ABoQApQCaAAacAKgAmAAA"
"pgCUAJsABqsAiQCSAACrAJIApwAAigCZAKwAAK0AowCiAAajAK4ApAAGkwCAAH8ABq8AYwBTAACwAJsApAAGgQCxAI0AAJAAjQCyAAGzALQAogAGrgCwAKQA"
"BpoAtQCiAAaQALIAmQABsgCsAJkAAY0AsQCyAAG2AJwAjgAApwC3AKsAALYAjgCKAAC4AJYAgAAGsQCBALkAAGMArwCqAACFALkAgQAAkwCRALoABrEAuwCy"
"AAGRAJUAvAAGuQC7ALEAAL0AqACcAACsALYAigAAvgCrALcAAL8AtwCnAAHAAJ8AqAABkwC4AIAABqwAsgDBAAGgAKUAjAAGnwC/AKcAAcAAvwCfAAHCAKAA"
"jwAGtQCzAKIABsMAuQCFAAClALUAmgAGqgDDAIUAALIAxADBAAGwAKYAmwAGwwDFALkAAMUAuwC5AAC7AMQAsgABxgDHAFwABr8AvgC3AAC9AJwAtgAAngDG"
"AFwABsEAyACsAAG0AK0AogAGrQCuAKMABqoAyQDDAADFAMoAuwAAywC+AL8AAL0AwACoAAC6ALgAkwAGzACqAK8AAMgAzQCsAADDAMoAxQAAzQC2AKwAALAA"
"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"
"AVcBAFcBNwFYAQDnAAYBJgEALQEIAVkBAFkBCAEkAQAnASYBWgEAWgFTAScBADwBOgE5AQA8ATkBTQEATQFUAU4BAJ0AVQEzAQAJAVsBIQEADwErATkBACUB"
"GgETAQBQARABBQEAXAEqAVcBAFABUQEQAQAZARwBKwEAUQFSAQsBAF0BNwE2AQBeAVkBJAEAOAEbAS0BAFMBXgEkAQBfASIBKgEAYAFWASABAFYBYQEsAQBY"
"ATcBXQEAHgE2AQcBAFsBIAEhAQAYAVsBCQEAWAFcAVcBBVIBzAD+AADMAFIBBgEAXgFTAVoBAF4BWgEmAQA5AVQBTQEAWwFgASABACIBXwElAQAqAVwBXwEA"
"YQFWAWABBCwBYQFiAQBdAVwBWAEFNQFjARYBAB0BFgFjAQAGAWQBJgEAZAEGAVIBAFkBXgEmAQBUAWUBVQEAWwFiAWYBAFsBZgFgAQBgAWYBYQEEYgFbARgB"
"AGYBYgFhAQBiAVIBLAEAYgEYAQQBAAQBUgFiAQBRAWcBUgEAZAFZASYBAC0BWQFoAQA2AR4BOAEAVAE5AWkBAGUBMwFVAQBpAWUBVAEAagEaASUBAGoBNQEa"
"AQBrAS0BaAEAZAFoAVkBAFIBZwFkAQBdATYBNQEAaQFsAWUBACsBaQE5AQBsAWkBKwEAXwFcASUBAGUBbAFtAQArAW0BbAEAHAFrASsBABwBHQFrAQBtAWsB"
"aAEAHQFjAWsBAGgBZAFtAQBrATgBLQEAZQE0ATMBAGUBUAE0AQBtAVEBZQEAKwFrAW0BAFwBNQElAQAlATUBagEAXAFdATUBAG0BZAFRAQBkAWcBUQEAYwE4"
"AWsBADUBNgE4AQBRAVABZQEANQE4AWMBAG4BbwFwAQZxAXIBcwEGdAF1AXYBBncBeAF2AQZ5AXcBegEGewF8AX0BBn4BfwGAAQaBAYIBfgEGewGBAYMBBoIB"
"fQF8AQaCAXwBfwEGggF/AX4BBoMBgAF8AQaEAX0BggEGewGEAYEBBoUBhgGHAQaIAYcBiQEGiQGKAYYBBokBhgGLAQaJAYsBjAEGhwGGAYoBBoYBjQGLAQaN"
"AYwBiwEGjQGOAYwBBo4BhQGJAQaPAZABkQEGkgGTAY8BBpQBkgGRAQaQAZUBlgEGkwGQAY8BBpMBlQGQAQaSAZUBkwEGlwGYAZkBBpoBmQGYAQabAZkBmgEG"
"mAGcAZ0BBp4BlwGZAQabAZ4BmQEGmwGcAZ4BBpwBlwGeAQafAaABoQEAoAGiAaMBAKEBoAGjAQCkAaEBpQEAnwGmAaABAKYBnwGnAQCoAaIBoAEApgGoAaAB"
"AKcBqQGmAQCjAaoBqwEArAGrAa0BAK4BrwGwAQCvAbEBsAEAsQGyAbMBAK0BqwG0AQCvAbUBsQEArwGsAbUBALIBsQG2AQCsAa0BsgEAtwG4AbkBALcBugG4"
"AQCyAa0BuwEAsgG7AbcBALkBsgG3AQC8AbgBugEAtwG9AboBAL4BrQG0AQC/AcABvQEAtwG/Ab0BALcBuwG/AQCtAcEBvwEArQG/AbsBAKsBqgG0AQCsAa8B"
"wgEArwHDAcIBAMQBrAHCAQDDAa8BxQEArwGuAcYBAK8BxgHFAQCrAccByAEAqwHEAccBAKMBqwHIAQC2AckBsgEAyQG1AawBALIByQGsAQCzAbIBuQEArgGw"
"AcoBALEBswGwAQCrAawBxAEAwQGtAb4BAMsBqgGjAQDLAaMBogEAywHMAaoBAM0BzgHPAQDNAdABzgEA0AHRAc4BAM4B0gHTAQDOAdMB1AEA1AHTAdUBANYB"
"0gHOAQDXAbwBugEAvgG0AaoBANgBvQG+AQDWAb0B2AEAvQHXAboBANcBvQHWAQDMAb4BqgEAvgG9AdkBAL4B2gHBAQC9AcAB2QEA2QHAAdoBAL4B2QHaAQC+"
"AcwB2AEA2AHbAc8BAM8B2wHcAQDcAd0BzwEAzwHdAd4BAM8B3gHNAQDMAd8B4AEAzAHgAdsBAMwB2wHYAQDLAd8BzAEAzwHOAdQBANgBzwHUAQDYAdIB1gEA"
"0gHYAeEBAOEB2AHUAQDhAdQB1QEA0QHiAdYBANYB4gHXAQDOAdEB1gEAogGoAcsBAOMB5AHlAQDjAeYB5AEA5gHnAeQBAOYB4wHoAQDmAegB6QEA5gHpAeoB"
"AOMB5QHrAQDjAewB6AEApAHtAe4BAKUB7QGkAQCnAe8B8AEA8QHyAfMBAPQB8gHxAQDxAfMB9QEA9QHzAfYBAPYB8wH3AQD2AfcB+AEAygH4AfkBAPYB+AH6"
"AQD7AfwB/QEA+wH9Af4BAPkB+AH3AQDyAfsB8wEA+wH8AfcBAPwB+QH3AQD7AfcB8wEA/wEAAvIBAPQB/wHyAQD/AQECAAIAAgIDAgACAAECAgIAAgDQAQQC"
"AwIAAwIFAtABAAMCAgIFAgAGAv4BBwIABgL7Af4BAAYCCAIHAgADAgYCAAIAAAIGAvIBAPIBBgL7AQAGAgMCCAIACAIDAgkCAAkCAwIEAgAKAgsCDAIADQIO"
"Ag8CABACpQGhAQARAhICEwIAEgIUAhMCBhICEQIVAgAUAhYCEwIApQEQAu0BAKEBFwIQAgAYAqYBqQEAGQIaAhsCBhwCqAEdAgCoAaYBHQIGpgEYAh0CABoC"
"HgIfAgaoARwCIAIAIAIcAh4CABoCHwIbAgapAfABGAIGGQIbAiECACICIwIkAgQlAiYCJwIEJwImAiICBCcCIgIoAgQmAiMCIgIEJAIoAiICBCkCKgIrAgQp"
"AiwCKgIEKgItAisCBB0CGAIuAgQuAhgCLwIEMAIuAjECBDACHgIuAgQxAi4CLwIELgIcAh0CBDACMQIyAgQeAhwCLgIEMwI0AjUCBDYCLQIsAgYqAikCNgIG"
"KgI2AiwCBjcCOAI0AgY1AjgCNwIGNQI3AjMCBjkCOgI7AgY5AjsCPAIGPQI+Aj8CBj0CQAI+AgZBAkICQwIGQQJDAkQCBkICRQJDAgZGAkcCSAIGRgJIAkkC"
"BkECRAJHAgZBAkcCRgIGSQJIAkMCBkkCQwJFAgZDAkgCRwIGQwJHAkQCBkoCOQI8AgZLAjoCOQIGTAI7AjoCBkoCPAJNAgZLAjkCSgIGTQI8AjsCBk0COwJM"
"AgZMAjoCSwIGTgJAAj0CBk8CPQI/AgZQAj8CPgIGTgJRAkACBk8CTgI9AgZRAlACPgIGUQI+AkACBlACTwI/AgakAZ8BoQEApAHuAcYBAKcBUgLvAQCnAfAB"
"qQEAUwJUAlUCBlQCUwJWAgZWAlMCVwIGVwJTAlgCBlkCVQJaAgZaAlUCWwIGWwJVAlQCBlwCXQJeAgBfAmACYQIAXwJdAlwCAGECYgJjAgBjAmICZAIAXwJc"
"AmACAGUCZgJnAgBoAmkCagIAaQJrAmwCAGgCagJtAgBnAm0CZQIAbgJvAnACAHECcgJzAgBuAnECcwIAbgJ0Al4CAG4CXgJxAgBxAnUCXgIAdgJuAnACAHAC"
"dwJ2AgB2AncCeAIAcgJ5AnMCAHQCbgJ2AgB0AnoCXgIAbwJuAnMCAHACbwJ7AgB8AnACewIAcgJ9AnkCAHwCeQJ9AgB8AmYCfgIAfAJ/AmYCAH0CfwJ8AgB9"
"AoACfwIAgQJ3AnACAHwCgQJwAgCBAoICdwIAfgKBAnwCAH4CZgKDAgB7AnkCfAIAhAKFAoYCAIcCiAKJAgCEAoYCiAIAigKLAowCAIYCigKIAgCIAooCjQIA"
"jgKIAoUCAIgCjQKFAgCPAoQChwIAjgKFAo8CAIoCkAKNAgCEAo8ChQIAkQKKAoYCAIcChAKIAgCQAooCjAIAkgKJApMCAIcCiQKSAgCUApUClgIAiQKIAo4C"
"AIkClwKUAgCOApcCiQIAlwKPApICAJcCjgKPAgCXApgClAIAlwKSApgCAJQCmAKTAgCUApMCmQIAlAKZApUCAJICjwKHAgCJApQCkwIAmgKbApwCAJ0CnAKe"
"AgCcApsCnwIAYQKgAl8CAJ0CngKhAgCgAp0CoQIAoAKhAl8CAF8CoQJdAgCiAqMCpAIApQKmAqcCAKICpAKeAgCeAqQCpgIApgKoAqcCAF0CoQKpAgClAp4C"
"pgIAoQKlAqkCAJ0CoAKqAgCrAqwCrQIArgKvAqsCALACsQKyAgCiAp4CnAIAsQKiApwCALICnAKfAgCwArICnwIAsgKxApwCAJ4CpQKhAgCdApoCnAIApgJy"
"AqgCALMCnwKbAgBpAmgCswIAtAJtAmcCALUCpAKjAgCzAmgCtAIAaAJtArQCALUCpgKkAgCmArYCfQIApgJ9AnICAGcCtwK0AgC0ArgCpgIAtAK3ArgCALkC"
"ugK7AgCrArsCrAIArgK5AmkCAKsCuQKuAgCxArACvAIAogKxAqMCALECvAKjAgCmArgCtgIAtQKjAp8CALQCtQKzAgC8Ap8CowIAsAKfArwCALMCtQKfAgCr"
"ArkCuwIAtQK0AqYCAGkCuQK6AgCrAq8CrQIArQKdAr0CAJ0CqgK9AgCdAq0CrwIAqgK+Ar0CAKoCYQK+AgCzApsCrgIAmwKaAq4CAJoCrwKuAgCqAqACYQIA"
"aQKzAq4CAJ0CrwKaAgC/AsACwQIFvwLCAsMCBcECwALEAgXDAsACvwIFxQLGAscCBcgCxwLGAgXJAsUCxwIFxgLKAsgCBcgCygLLAgXJAswCxQIFzQLOAs8C"
"Bc8C0ALRAgXNAs8C0QIF0QLOAs0CBdIC0wLUAgbTAtUC1AIG0wLWAtUCBtYC1wLVAgbXAtgC1QIG1wLSAtgCBtkC2gLbAgbZAtsC3AIG1ALVAtgCBtIC1ALY"
"AgbTAt0C3gIG0wLeAtYCBtcC2gLZAgbSAt0C0wIG1gLeAtoCBtYC2gLXAgbSAtkC3QIG1wLZAtICBt0C2QLcAgbfAuAC4QIG3wLiAuACBuEC4wLkAgbhAuQC"
"3wIG5QLfAuQCBuYC5wLoAgDnAukC6gIA6wLsAu0CAO4C7wLwAgbuAvAC8QIG7wLuAvICBvMC9ALyAgb0Au8C8gIG9QLxAvACBvUC8AL2Agb3AvgC+QIG+gL4"
"AvcCBvkC+AL1AgZcAl4CegIAYQJgAmICAIMCZgJlAgBpAmwCagIAdwL7AngCAHcCggL7AgCRAosCigIAhgKFApECAJMClAKWAgCTApgCkgIA/ALbAv0CBv4C"
"/wIAAwbeAt0CAQMG3gIBAwIDBtsCAwP9AgbbAtoCAwMG2gLeAgMDBt0C3AIBAwb/AgQDAAMG3gICAwMDBtwCBQMBAwYGA+IC3wIGBgPfAuUCBuMCBwMIAwbj"
"AggD5AIG5QLkAggDBgkD8ALvAgYKAwsDDAMGDQMOAw8DBg8DDgMQAwYPAxADEQMGEQMQAxIDBhEDEgMTAwYTAxIDFAMGEwMUAxUDBhUDFgMXAwYVAxcDGAMG"
"GAMXAxkDBhkDGgMbAwYbAxoDHAMGGwMcAwoDBh0DHgMfAwYeAyADHwMGHgMgAyEDBiEDIAMiAwYhAyIDIwMGIwMiAyQDBiMDJAMlAwYlAyQDJgMGJQMmAycD"
"BicDJgMoAwYoAykDKgMGKQMrAywDBikDLAMtAwYtAywDLgMGLQMvAy4DBi8DLgMwAwYvAzEDMAMGMQMwAzIDBjEDMgMzAwYzAzIDNAMGMwM0AzUDBjUDNAM2"
"AwY1AzYDHQMGNwM4AzkDBjgDOgM5AwY4AzsDOgMGOwM8AzoDBjsDPQM8AwY9Az4DPAMGPQM/Az4DBj8DQAM+AwY/A0EDQAMGQQNCA0ADBkEDQwNCAwZDA0QD"
"QgMGQwNFA0QDBkUDRgNEAwZFA0cDRgMGRwNIA0YDBkcDSQNIAwZJA0oDSwMGSgNMA0sDBkoDTQNMAwZNA04DTAMGTQNPA04DBk8DUANOAwZPA1EDUAMGUQNS"
"A1ADBlEDNwNSAwZTA1QDVQMGUwNUA1YDBlYDVANXAwZWA1cDWAMGVgNZA1gDBlkDWANaAwZZA1oDWwMGWwNaA1wDBlsDXQNcAwZdA14DXwMGXQNeA2ADBmAD"
"XgNhAwZhA2IDYwMGYwNiA2QDBmMDZANlAwZlA2QDZgMGZQNmA1MDBmcDaANpAwxoA2oDaQMMaANrA2oDDGgDbANrAwxtA24DbwMMcANxA3IDDHEDcwNyAwx0"
"A3UDdgMMdAN3A3UDDHgDeQN6Awx7A3wDfQMMfAN+A30DDH4DfwN9Awx+A4ADfwMMgQOCA4MDDIIDhAODAwyCA4UDhAMMhQOGA4QDDIUDhwOGAwyIA4kDigMM"
"iQOLA4oDDIwDjQOOAwyPA5ADkQMMkgOTA5QDCpUDlgOXAwCWA5UDmAMAmQOaA5cDAJYDlwOaAwCVA5cDmQMAmgOYA5YDAJsDnAOdAweeA5wDmwMHngOfA5wD"
"B6ADoQOiAwCiA6MDoAMApAOhA6UDAKIDpQOhAwCgA6QDoQMApQOiA6MDAKYDpwOoAwepA6YDqAMHqQOoA6oDB5OLATxGOhSRXRSf0W6ODxaGMA2N3kTEwvST"
"QzsTmUOa9RkSlMWK1Up+se6jS0HYP/WM90UrT+6R3y+kYmiYpjZ9afShK1/8EEOkaRxiMuOfqlvG+J+hGj9hxuCWPjHrXNedkk/Ek/GYfhcUMZSr4Ev1aVue"
"pBBeYISeoxFjaeiZlhbJ0kSo9Tm5WKajdFfZIIij5V9Lzy21aGOTqaamIhhxz72mdRuzoDqi3DzIqsG1+ibEXm6x8kBWw/i2mhhV30G3DhiGMru0XnX0+rGr"
"pmtAz7Gz9xvGqDK2hGKKk0+2pUQ9Qju2b3jm5zq+gnfgAmTCYnZtyvS0DhqtkwG3DhMDwLG71Bf/VkG6+g8sSZ64BhjcQN/0TFc/TvLIQh9Xxc/a/39O5ur0"
"FVlDLJDKdiFFrm/M2mIung1FKxMvRm1g+RFuabNbChhsMbUywhnB3sMmlxy3rV1FyhXZW2PV+RHGs1QwaBkRxHcJjGMGmQ5GIRhbMjI2aHdLy8xFxmKhk7Q2"
"bBOSs2JAbnioAgY9u38t63tJckxfaWUy1VzbIxE6dhO1k5MhH1oUL2JctlfMHx41PWQqoTdkGlAqUj1GHGIrqOlKfRUktVBK9hvBk8pJQhy3yvFGMGu6ueNS"
"al66paBSXBoondtGLDeEWrxHJj8ePWVLnXMU/o9Iend+6NFTq2oMzmNNRUC8rmlllxPmX3Vc5DyOx2RbJkQ5QidjBkuck9FfWVq4AZBf/V1aEZ5dihY+0eJk"
"YR6sk99e7TsmqnVWyzdKWTVnxzdZWsNchF6E0CJzEE5ftdRtMh/woAJkyRUn9Kdyez6uyJRnijBxaRhnzhbfLxdzFkYmSOZq4zobmR5ujxT3z9xwbBgLMNxm"
"kyN6RulnLBP9RjtlPiFhTeD8pU5TZmTSuEWlcuYufEZ4cUun9kgzZk6q6kN6aU5lKTrJZdpQpTQcc+FlmRnsZao/6Bq3eT7EiCl6ejnUNRtvfc4o0TeIfDZA"
"fS0CekfMmDqEe8+arDqiZbmsuinQc1ysQjZ9ctyYGhoJZsq8axnNez2kvjgZbsRINRm/eWNKSBF5c/HPYRh/fCmidA4lZsixsBppeLvUkxrRfE8qtTbkelNV"
"sztpaGkyf0UYcLSiFznpcuLZuje7e+o2WSWIet02lyi+cUnJbyTSccjH9SeZem5ReERkZbRc9jm8bJWZyzhhZl5eShNKZrAejBbHe3WgXBNKZmNMdxYgdf6z"
"+BZzdVjYlBbxemtZ30/gisBjsBhJl51XMx5wkWtKQ1l1ir9W61xkl9yWgT6LluKzOVfEjWilz05Gi9Sq3yIhkZEMUlPZhdj5kmDJiD0ce1iviHJIyl7elrHm"
"zFU+hya0zV0Gl+U5L3ZNj1MxiHaQh1vHQnYliNFLv1yQjxJNqGNSl/++93Folh9SeFyTlhJLOCJ9g03R5C39gM4qOzF6gfQyGUdsgTPUSUcXgbjYe09Bhdkd"
"2k2ThB+uAyABhmw7ayO2jGlNUVfpi2YeuE3HhVQclFYYiChOWUzHisqww1Wmjb7jb1YUiJayIkt0iejiQEnkhsG2ZiHrltbCbyS2i/wm6iNYi85GLSfqjNZY"
"9yHJltr//iUuktr/HSfOie/HFicNjDnJAya+kvK+fSXglk+qKhYukzWxgyj/lPQ84SEUj446ARLXlnI3SiX9lvpDViMll0YuQiLtltr/TyXglgAAAQACAAAD"
"AAEAAAAABAAFAAYAAAcAAAACAAAIAAMAAAAACQAKAAgAAAsAAwAIAAAMAAcAAgAAAwALAA0AAAYAAwANAAADAA4AAQAACAAPAAkAAAQABgAQAAARAAIAAQAA"
"CgASAAgAAAkAEwAUAAAVABEAAQAADAACABEAABQACgAJAAAWAA8ACAAAAAAHAAgAAAgAFwALAAAIABIAFwAACQAPABMAABAABQAEAAAYAAYADQAAGQAQAAYA"
"ABoAEQAVAAYGAA4AAwAAAQAaABUAAAEADgAaAAAGABgAGQAABgAFABsAABwADgAGAAAbABwABgAADwAWAB0ABggABwAWAAAcAB4ADgAGGgAfABEABh8AIAAR"
"AAYOAB4AGgAGIQANAAsACSIAGAANAAkcABsAIwAGGQAkABAAACAADAARAAYdABMADwAGBwAMACUABgcAJQAWAAYZABgAIgAAIQAmAA0ACQ0AJgAiAAkLABcA"
"JwAAKAAZACIAAAUAKQAbAAAlAAwAIAAGJQAdABYABh4AKgAaAAYhAAsAJwAAGwApACMABh4AIwAqAAYeABwAIwAGJgAoACIAACsAEwAdAAYdACwAKwAGHQAt"
"ACwABh0AIAAtAAYgAB0AJQAGLgAXABIAACoALwAaAAYvAB8AGgAGJwAmACEAADAAKAAmAAAuADEAFwAAIwAyACoABioAMgAvAAYwACYAJwAAJwAXADEAATMA"
"JAAZAAA0ACwALQAGNQAUABMABh8ANgAgAAYvADcAHwAGKQA4ACMABjkAEwArAAYyADoALwAGOAAyACMABigAMwAZAAE7ADcALwAGKAA8ADMAASAAPQAtAAYw"
"AD4AKAAAPwAkADMAAD8AMwA8AAA6ADsALwAGOABAADoABjIAOAA6AAYxAEEAJwABQgAwACcAABIAQwAuAABCAD4AMAAAQQBCACcAACwAOQArAAZBADEARAAB"
"PgA8ACgAAUUAOAApAAZGADEALgAAQAA7ADoABkYARwAxAABIAD8APAAANAA5ACwABkkALgBDAAA3ADYAHwAGRQBAADgABj0ANAAtAAZHAEQAMQAAOQA1ABMA"
"BkkARgAuAABIADwAPgABSgA/AEgAAEAASwA7AAZAAEwASwAGOwBNADcABkAARQBMAAZIAD4ATgABTgBKAEgAAUIATgA+AAA/AEoATwAASwBNADsABkEARABH"
"AAFQAEsATAAGNABRADkABj0AUQA0AAY9AFIAUQAGQQBTAEIAAEIAUwBUAABPAEoATgAAVQBCAFQAACAANgA9AAZLAFYATQAGSwBQAFYABkcAUwBBAABPAE4A"
"QgAATwBCAFUAADkAVwA1AAZYAE0AVgAGUgBZAFEABkkARwBGAABaAD8ATwAAUwBbAFQACTYAUgA9AAZRAFcAOQAGRwBcAFMAAE0AXQA3AAZYAF0ATQAGXQA2"
"ADcABl4AUABMAABfAFYAUAAGWQBgAFEABmEAVwBRAAZiAFQAWwAJWQBSADYABlsAUwBcAAliAE8AVQAAYwBaAE8AAFQAYgBVAAlkAF8AUAAAXwBYAFYABmQA"
"UABeAABRAGAAYQAGYgBjAE8AAF0AZQA2AAZmAF0AWAAAQwBnAEkAAFsAYwBiAAA2AGUAaAAGYABZAGEAAEkAZwBpAABeAFoAagAARwBJAFwAAGMAWwBcAABl"
"AF0AawAAZQBsAGgAADYAaABsAABdAGYAawAAbABZADYAAGwAaQBZAABqAFoAYwAAawBsAGUAAFcAZwA1AABeAGoAZAAAYwBmAF8AAFgAXwBmAABJAGkAXAAA"
"VwBhAGcAAGEAaQBnAABZAGkAYQAAYwBcAGYAAGMAXwBkAABpAGYAXAAAbABrAGYAAGYAaQBsAABqAGMAZAAAbQBuAG8ABnAAcQByAABxAHAAcwAAcwB0AHEA"
"AHUAdgB3AAB4AHkAegAAewB8AHYAAHgAdgB8AAB2AHgAdwAAcQB7AHIAAHEAfQB7AAB+AH8AgAAAfgCBAH8AAIEAggB/AAB5AHwAewAAggB6AHkAAH0AeQB7"
"AAB9AIAAeQAAeQB/AIIAAH8AeQCAAABzAIMAdAAAhACFAIYAAIUAdwCEAACBAIcAiAAAiACEAIYAAIgAhwCGAACJAIgAhAAAigCLAIwABHEAdACNAASNAI4A"
"cQAEjwCQAJEABo8AkQCSAAaSAJEAkAAGkgCQAI8ABpMAcAByAACTAJQAdQAAcwCVAIMAAJYAlwCYAAaXAJYAmQAGmgCYAJsABpsAmACXAAacAJ0AngAAnACf"
"AKAAAJwAoACdAAChAKIAowAAowCkAKEAAJ8ApQCmAAClAKcAnwAAqACfAKYAAKUAqQCmAACmAKIAqgAAqQCiAKYAAKsArACtAACrAK4ArAAArACuAK8AAKwA"
"rwCwAACrALAArwAAsQCuAKsAAKsArQCwAACsALAArQAAsgCzALQAALIAtAC1AACeALIAnAAAsgC1AJwAALUAtgC3AAC3AKUAuAAAuAC1ALcAALkAsgC6AACz"
"ALUAtAAAtQC4AJwAAKQAowC5AACjALcAtgAAtwCpAKUAALoApAC5AAC1ALMAtgAAowC2ALMAALkAowCzAACyAJ4AugAAswCyALkAALsAvAC9AAW+ALwAuwAF"
"vwDAAMEABcIAwQDAAAXDAMQAxQAGxgDHAMgABsYAyADJAAbKAMkAyAAGygDIAMsABswAzQDKAAaqAKIAoQAApADOAKEAAKYAqgCoAACrAK8AsQAAzwDQANEA"
"BtAAzwDSAAbDAMUA0wAG1ADIAMcABg=="
;
static const char* CARDATA_SPORT2 =
"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"
"QwBJAABLAFgATgAAJQBZAC4ABjEAWgAyAAYyAFsAUgAAPwBXAFUAAEMAXABJAABcAE0ASQAAPABdAF4AAFAAXwBDAAAEAGAABgAAYQAWAAgAAGEACAAYAABA"
"AGIAIgAAWABjAE4AAEoAPABeAABkAGUARQAABABFAGAAAGYAYQAYAABnAGIAQAAAaABYAEsAAEMAXwBcAAAYAAYAZgAAaQAjAEYAAFEAXQA8AAAyAGoAawAG"
"bABtAEgAAFIAbABIAABuACEARwAGbgBWACEABkgAbQBvAABiAEsAIgAAZAAdAEEAAEEATQBwAABkAEUAHQAAYwBPAE4AAE8AYwBxAAAWAGkARgAABgByAGYA"
"AFEAcwBdAAA9AHQAUwAGUwB0AEcABnUALQA+AAYZACQANQAGSwBiAHYAAEsAdgBoAABgAEUAZQAAIwB3AFEAAHcAeABRAABaAGoAMgAGHgBUAB8ABkcAeQBu"
"AAZ6AFYAbgAGPgBVAHUAAHoAQABWAABeAHsASgAATQB8AH0AAE0AXAB8AABxAF8AUAAAcgAGAGAAAGkAdwAjAABbAGsAUgAGWwAyAGsABmwAUgBrAAY9AH4A"
"dAAGdAB+AH8ABn4AgAB/AAZsAIEAbQAGbQCBAIIABmgAdgCDAAmDAIQAaAAJhQBcAF8AAF0AhgBeAAaHAGkAFgAAcwBRAIgAAIgAUQB4AABAAHoAiQAAUABP"
"AHEAAH0AcABNAACFAHwAXAAAZwBAAIkAAF4AigB7AABxAIsAXwAAZABBAHAAAIwAZgByAACNAI4AdwAAaQCNAHcAAIgAjwBzAABrAIEAbAAGkACEAIMABEoA"
"kQBXAAVKAJIAkQAFewCSAEoABYoAkgB7AAB8AIUAkwAJXgCGAIoABmYAlABhAACVAGQAcAAJjACUAGYAAF0AlgCGAAZzAJYAXQAAlwCOAI0AAI4AmAB3AAB4"
"AHcAmAAAmQCIAHgAAFcAkQBVAAB8AJoAfQAAiwCFAF8AAHAAfQCbAAllAJUAnAAAZQBkAJUAAJYAnQCGAAaeAI0AaQAAnwCWAHMAAI8AiACZAAAkAKAANQAG"
"oQCJAHoAAGIAZwB2AASQAIMAdgAEogCLAHEAAHIAYACjAACjAGAAZQAAZQCcAKQACZcAjQCeAAClAHMAjwAAdQCmAC0ABoQApwBYAAB8AJMAmgAJiwCoAIUA"
"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"
"WwEGZwE8AToBAFUBaAFMAQb8ABsBFAEJTQFMAWgBBksBTQFpAQYdATcBQgEANAFqAUgBCRMBawFXAQBpAUUBSwEGaAFsAW0BBmwBaAFVAQZHAW4BXwEARQFv"
"AT0BBm0BTQFoAQZWAWwBVQEGWQFuAUcBCXABXQFcAQAwAWsBEwEAaQFvAUUBBh8BLAEaAQBnAToBYAEAbwFxAT0BBk0BbQFpAQb/AGUBcgEGQAFmAUEBAEgB"
"cwFEAQlyAS0BKgEAPwE5ARwBAFoBXwFmAQBDAU8BSQEJbwFpAXQBBlgBdQFWAQYaAS0BZAEASQFuAVkBCXMBYAFEAQl2AWkBbQEGZAF3AT8BAHgBQQF5AQA9"
"AXEBegEGGAF7AWEBBhcBewEYAQZqAXwBSAEJSQF9AW4BCXwBcwFIAQlPAX4BSQEJOwF/AYABBm8BdAFxAQaBAXQBaQEGaQF2AYIBBoMBdgFtAQZsAYMBbQEG"
"LQF3AWQBADwBZwGEAQBWAYMBbAEGIQFbAVgBAEIBhQEdAQA0ATMBagEAYQGGAQABBmkBggGBAQYtAXIBhwEAWwF1AVgBAB0BhQFbAQZBAWYBeQEAXwF5AWYB"
"AE8BUAGIAQn3AOMAGAEGgQGJAXQBBioB/wByAQZ1AYoBVgEGMwE5AYsBAGoBjAF8AQmNAV4BXQEAOwE9AXoBBnQBjgFxAQZ7AY8BYQEGAAGGAWIBBmMBZQFG"
"AQaKAYMBVgEGkAE5AT8BAEIBkQGFAQBBAXgBQgEGYAFzAZIBCYgBUAE8AQCCAZMBgQEGgwGUAXYBBnYBlAGCAQZ3AZUBPwEAeAGRAUIBBn4BfQFJAQk8AYQB"
"lgEAdAGJAY4BBpMBjgGBAQYMARcB4wAGZwFgAZIBAEYBAAFjAQYzAYwBagEAggGUAZMBBpcBcgGYAQByAZcBhwEAbgF9AZkBAJoBfgFPAQmOAXoBcQEGCwGb"
"AQwBBnUBnAGKAQYtAYcBdwEAlQGQAT8BAHgBnQGRAQZeAXABXAEAiAGeAU8BCXoBjgGfAQY7AXoBnwEGnwGgATsBBgwBoQEXAQZbAZwBdQEAlQGiAZABAIUB"
"owFbAQVzAXwBpAEJiAE8AZYBAI4BiQGBAQalAaYBoAEGnwGlAaABBpQBpwGTAQaQAYsBOQEAbgGoAXkBAJIBcwGpAQAXAaoBqwEGewEXAasBBmUBmAFyAQaD"
"AYoBlAEGrAGMATMBAIwBrQF8AQl8Aa0BrgEJVwFrAU4BAE8BngGvAQmfAY4BsAEGoQGqARcBBqcBsQGTAQawAaUBnwEGsgF7AasBBrMBogGVAQSWAbQBiAEA"
"igGnAZQBBpkBfQG1AQB5AbYBnQEAngG3Aa8BALcBngGIAQCSAakBZwEAjgGTAbEBBrgBsAGOAQarAbkBsgEGlwGYAboBAJcBuwGHAQSFAbwBowEFiwGsATMB"
"AH0BfgG9AQl5Ab4BtgEATgFrAb8BAAwBwAGhAQaxAacBwQEGsQG4AY4BBpEBvAGFAQB4AXkBnQEAqAFuAZkBAMIBwwGtAQmaAU8BrwEJtAHEAYgBANkADAHj"
"AAbBAacBxQEGxgGYAWUBBqoBxwGrAQaKAcUBpwEGdwGHAbsBBIsBkAHIAQCsAa0BjAEAqAHJAXkBAG4BeQFfAQALAcoBywEGmwHMAQwBBrEBwQHNAQayAY8B"
"ewEGzgGqAaEBBrEBzQG4AQa4Ac8BsAEGsAHQAaUBBqUB0AHRAQbSAXcBuwEEWwGjAZwBAKIBswGQAQCkAXwBrgEJcwGkAdMBAHMB0wGpAQC3AYgBxAEAcAGN"
"AV0BAI8BhgFhAQZlAWMBxgEGzgHHAaoBBscBuQGrAQbPAdABsAEGpgGlAdEBBtEB1AGmAQbSAZUBdwEEvAHVAaMBBdYByAGQAQCRAdcBvAEA2AF+AZoBCaYB"
"1AGgAQaXAboBuwEAhAG0AZYBCaAB2QF/AQayAdoBjwEGwAEMAdsBBooB3AHFAQaYAcYBugEGswGVAdIBBNUB3QGjAQW9AbUBfQEAvQF+AdgBCa8BtwHeAQCA"
"AcoBCwEGDAHfAdsBBtwBwQHFAQaKAZwB3AEGwwGuAa0BCb0B2AG1AQDgAdMBpAEApAGuAeABANMB4QGpAQleAb8BcAEAvwFeAY0BAKAB1AHZAQacAd0B4gEA"
"nAGjAd0BALMB1gGQAQCsAeMBrQEArgHDAeABAJoBrwHkAQCEAeUBtAEJ5gG/AWsBAL8BMAFOAQC/AecB6AEA5gFrAegBADABvwHoAQDZAAkBCwEG6QHPAbgB"
"BsIB6gHDAQDDAeoB4AEArwHeAeQBAMkBvgF5AQBwAb8B6wEAcAHrAewBAOwBjQFwAQC/Ae0B6wEA7gHvAY0BAI0B7wG/AQDvAecBvwEAawEwAegBAM0B8AHp"
"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"
"BlcCYgJYAgZjAmQCZQIGYwJlAmYCBmMCZgJnAgZjAmcCaAIGYwJoAmkCBmMCaQJqAgZjAmoCawIGYwJrAmwCBmMCbQJuAgZjAm4CZAIGbwJwAnECBm8CcgJw"
"AgZvAnMCcgIGbwJ0AnUCBm8CdgJ0AgZvAncCdgIGbwJ4AncCBm8CeQJ4AgZvAnoCeQIGbwJxAnoCBnsCfAJ9AgZ7An4CfAIGewJ/AoACBnsCgQJ/AgZ7AoIC"
"gQIGewKDAoICBnsChAKDAgZ7AoUChAIGewKGAoUCBnsCfQKGAgaHAogCiQIGhwKKAokCBooCiQKLAgaLAokCjAIGiwKNAowCBo0CjAKOAgaPApACkQIGjwKS"
"ApECBpICkwKRAgaTApEClAIGkwKUApUCBpMClgKVAgaWApUClwIGlgKXApgCBpgClwKZAgaYApkCmgIGmgKZApsCBpoCnAKbAgacApsCnQIGnAKdAp4CBp4C"
"nwKdAgagAqECogIGogKjAqECBqICpAKjAgakAqMCpQIGpAKlAqYCBqYCpQKnAgamAqcCqAIGqAKnAqkCBqgCqQKqAgaqAqkCqwIGqgKsAqsCBqwCqwKtAgas"
"Aq4CrQIGrwKwArECBq8CsAKyAgayArMCsAIGsgKzArQCBrQCtQKzAga0ArYCtQIGtgK3ArUCBrYCuAK3Aga4ArkCtwIGugK7ArwCBroCuwK9Aga9ArsCvgIG"
"vQK/Ar4CBr0CvwLAAgbAAr8CwQIGwALCAsECBsICwwLBAgbDAsQCwQIGwwLFAsQCBsUCxgLEAgbFAscCxgIGxQLHAsgCBsgCxwLJAgbIAsoCyQIGyALKAssC"
"BssCygLMAgbLAs0CzAIGzQLOAswCBs0CvQK6AgbNAsACvQIGzQLCAsACBs0CwwLCAgbNAsUCwwIGzQLIAsUCBs0CywLIAgbPAtAC0QIGzwLRAtICBtIC0QLT"
"AgbSAtMC1AIG0gLUAtUCBtUC1ALWAgbVAtYC1wIG1wLWAtgCBtcC2ALZAgbXAtkC2gIG2gLZAtsCBtoC2wLcAgbaAtwC3QIG2gLeAt0CBt4C3wLdAgbeAuAC"
"3wIGzwLSAtUCBs8C1QLXAgbPAtcC2gIGzwLaAt4CBs8C3gLgAgbhAuIC4wIG4gLkAuMCBuIC5QLkAgblAuYC5AIG5QLnAuYCBucC6ALmAgbnAukC6AIG5wLq"
"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"
"AABMAFIAbgAGYQBsAFkAAGEAYwByAABzAHQARAAGcAB1AFoACVcAOQBbAAA5AFcAZAAAUABrAHMABkoAVQBdAABcAGoAcAAAdgA5AEEAAHYAQQBOAAAcADcA"
"RAAGOQB2AFsAAGIAdwBtAAZuAHgAXgAAYwB5AHIACUQAdABpAAZxAF4AeAAEWABbAGUAAFoAeQBjAAldAHcAYgAGWwB2AGUABlAAcwBEAAZpAHQAegAGawB7"
"AHMABlUAfABdAABlAHwAVQAFdgBOAGQAAE4AOQBkAAB1AHAAagAAVAB1AGoAAFoAdQB5AAlhAHIAbAAAbgBxAHgAAGoAYABUAABoAGQAWwAAXwB9AFIABmwA"
"eQB+AAByAHkAbAAAfABlAH8AAG4AUgB9AAZuAIAAcQAAaAB2AGQAAIEAegB0AACBAH4AdQAAggBsAH4AAHUAfgB5AAB/AF0AfAAAdgB/AGUAAHUAVACBAABz"
"AH4AdAAAXQB/AHcABn4AgQB0AAB9AIAAbgAAgABvAHEAAIMAfwB2AABzAIQAfgAAewCEAHMAAHcAfwCDAAB2AGgAgwAAhACCAH4AAGgAbACFAACDAGgAhQAA"
"hAB7AIIAAIYAhwCIAAaGAIgAiQAGhgCJAIoABosAjACNAAaLAI0AjgAGiwCOAI8ABpAAkQCSAAaQAJMAlAAGkACSAJMABpUAlgCXAAaVAJgAlgAGlQCZAJgA"
"BpoAmwCcAAaaAJ0AnAAGmgCdAJ4ABpoAnwCeAAafAKAAngAGnwChAKAABp8AoQCiAAafAKMAogAGnwCjAKQABqQAowClAAakAKYApQAGpgCnAKUABqYAnwCa"
"AAamAKQAnwAGqACpAKoABqgAqgCrAAarAKoArAAGqwCsAK0ABqsArgCtAAauAK8ArQAGqACrAK4ABrAAsQCyAAawALMAsQAGswC0ALEABrMAtQC0AAa1ALYA"
"tAAGtgC3ALQABrYAswCwAAa2ALUAswAGuAC5ALoABrgAuQC7AAa7ALkAvAAGuwC8AL0ABr0AvAC+AAa9AL4AvwAGvQDAAL8ABrgAuwC9AAa4AL0AwAAG"
;
static const char* CARDATA_HATCH2 =
"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"
"ACkAeABUAAYuAGUAgQAAiwBrAIEABIwAkwBrAACOAHEATQAAkwBYAGAABHIAWACUAARtAGIAlQAAkQB0AIIACYMAaACWAAloAHUAlwAJgwA3AGgACWoAUQCJ"
"AAZSAJgAdgAGmQBxAI4AAGsAkwBgAAA0AI8AcwAAcwB6AGEAAGEAIwBzAABjAJEAmgAAYwB0AJEAAJAAmwCRAAmCAJwAkAAJfQCcAIIAAJYAaACXAAmSAJcA"
"dQAAkgCdAJcAAIgAngCJAAZlAJ8AgQAAXwCgADkABpwAfQChAAChAH0AhAAJhACiAKEACZcAowCWAABwAKQAoAAGjQBxAJkAAGMAmgB7AAALAKUAogAJfgCl"
"AAsACX4AgACmAACWAKMAgwAAhQCfAGUABjkAoACnAAY5AKcAigAGeQBOAKgABU4AfACoAAVzAKkAegAAogCqAKEAAIQACwCiAAmXAJ0AowAAogClAKsAAH4A"
"pgClAAmmAIAAgwAAjQCsAGwABpQAWACTAAQ0AFkAjwAAlQB/AG0ACZoAkQCtAACtAJEAmwAAnACbAJAAAJwAoQCuAACmAKsApQAAhQB2AK8ABoEAnwCwAACL"
"AIEAsAAEjgB5ALEAALEAeQCoAAWrAKoAogAAqwCmAIMAACcAdwApAAayALMAtAAGeAB3ALUABlcAtgCkAAZXAGwAtgAGjQC3AKwAAHsAuAB8AAC5ALoAuwAG"
"sgC8AL0ABrMAsgC9AAasALYAbAAGtwCNAJkAAL4AjgCxAACPAHIAvwAAwABiAHoACa0AmwDBAACbAJwArgAArgDBAJsAAKoArgChAACjAMIAgwAAwwC5ALsA"
"BocAiABRAAafAMQAxQAAsACfAMUAALAAxQCMAAB6AMYAwAAAvQC8AMcABl8AcACgAAbIAMkAygAGygDJAIgABrwAtQDHAAaYALMAywAGjgC+AJkAAL4AzACZ"
"AAByAJQAvwAAfwDNAJIACYMAwgCrAAC9AM4AswAGmADLAHYABnAAVwCkAAaLALAAjAAAkwCMAM8AAKkAxgB6AADQAK0A0QAAwQDRAK0AANIAzgC9AAaFAK8A"
"nwAGuwC6ANMABsgAugDJAAbTALoAyAAGzAC3AJkAALgAqADUAACoAHwA1AAFuADUAHwAAJoArQDVAAC9AMcA0gAGywCvAHYABs4AywCzAAbWANcApwAG1wDW"
"ALkABswA2AC3AADZANoAkwAE2gCUAJMABNAA1QCtAACdAM0AowAAwgCjANsAALkA3AC6AAaJAF4AagAGhwDKAIgABo8A3QCpAAC3ANgArAAAjwC/AN0AAHMA"
"jwCpAADbAKMAzQAAxwC1AHcABqwA3gC2AAaTAM8A2QAAlADfAL8AAK4AqgDBAADDANcAuQAGjADgAM8ABuEAvgCxAACoAOEAsQAFwADiAJUACeMAmgDVAAmS"
"AM0AnQAJwgCqAKsAAMcA5ADSAAafAK8AxAAGuQDWANwABs8A4ADZAADdAOUA5gAA0QDnANAAAOgA5wDRAACqAOkAwQAAtgDqAOsABowAxQDgAAbhAOwA7QAA"
"qQDmAOIAAGIAwACVAAmkALYAoAAGvgDhAO0AAO4A4wDVAAnQAO4A1QAArwDvAMQABtwA8AC6AAYnAPEAdwAG8gDFAMQAAMUA8wD0AAbFAPQA4AAGlADaAN8A"
"AMYA4gDAAAC6APAAyQAG8ACIAMkABvUA8gDEAADyAPMAxQAG4QCoAOwAAMYAqQDiAAD2ALgAewAJmgDjAHsACeMA9gB7AAnkAPcA0gAGdwD4AMcABswAvgDt"
"AADQAOcA7gAA+QDbAM0AAO4A+gDjAAnuAOcA+wAA0gD8AM4ABvgA/QDkAAbLAP4A7wAGxwD4AOQABqkA3QDmAAD6AO4A+wAJ5wD/APsAAAABXgCJAAavAMsA"
"7wAG7wABAcQABrYA6wCgAAbqALYA3gAG2AACAawAANkA3wDaAADCAAMBqgAA9wD8ANIABqwAAgHeAAbYAAQBAgEABQF/AJUACc4A/ADLAAbxAPgAdwAGzAAG"
"AdgAALgABwHsAADiAAUBlQAJ6AD/AOcAAPkACAHbAADEAAEB9QAA7QAGAcwAAOIA5gAJAQDiAAkBBQEJCgEDAcIAAAoBwgDbAAD8AP4AywAGCwGIAPAABvUA"
"8wDyAAbfAAwBvwAA+QDNAH8ACdEAwQDoAAACAQ0B6gAGDgHgAPQABu0ADwEGAQDsAA8B7QAAqAC4AOwAABAB3QC/AAC4APYABwEJwQDpAOgAAOQA/QD3AAb4"
"ABEB/QAG3AALAfAABgcBEgHsAAATAf8A6AAAFAETAegAAPgA8QARAQYGAQQB2AAAFQHfANkAAOwAFgEPAQAJARcBBQEJfwAFAfkACRMB+wD/AAAYAQsB3AAG"
"GAHcANYABt4AAgHqAAbgAA4BFQEA2QDgABUBAPsAGQH6AAn9ABoB9wAGoADWAKcABgwBEAG/AAD8APcA/gAG7wAbAQEBBhwBHQEeAQYLARwBHgEG6wAfAdYA"
"BgEBIAH1AAD1ACAB8wAG4wD6APYACfoAIQH2AAn3ACIB/gAGIwHfABUBABoBIgH3AAYbASQBAQEGJQEmAQwBAPoAGQEhAQnpABQB6AAAEQEaAf0ABicBIwEV"
"AQASARYB7AAAIQEHAfYACaoAAwHpAAAYASgBCwEGDQEpAesABg8BKgEGAQArAfsAEwEAIgEsAf4ABhEBLQEaAQbWAB8BLgEGAQEkASABAN8AIwElAQASAQcB"
"LwEA+QAFATABCdYALgEYAQbqAA0B6wAGDgEnARUBAN8AJQEMAQAJAeYAFwEAMQEKAdsAAF4AAAEyAQb+ACwBGwEGHwEzAS4BBqAA6wDWAAYSATQBFgEHEgEv"
"ATQBAAUBFwE1AQkvAQcBIQEJ+QAwATYBCQ0BNwEpAQbzADgB9AAGngA5AYkABgIBOgENAQYEAToBAgEAJwEOATsBACMBJwE7AQY7ATwBIwEG+QA2AQgBANsA"
"CAExAQAWAT0BDwEANAE9ARYBByIBGgE+AQZeADIBhgAGhgA/AfEABiUBIwE8AQAlAUABJgEANQEwAQUBCQgBQQExAQDxAD8BEQEG/gAbAe8ABjgBQgH0AAYG"
"ASoBBAEAEAHlAN0AABkBQwEhAQkZAfsAQwEJQwH7ACsBAD4BLAEiAQYtAUQBGgEGPgFFASwBBhgBLgEoAQYoAUYBHAEGJAFHASABACABOAHzAAYOAfQAQgEG"
"PQFIAQ8BAEMBSQEhAQmeAEoBOQEGOQEAAYkABjkBSwEAAQbrACkBHwEGSQEvASEBCRQBKwETAQBEAT4BGgEGhgDxACcABoYATAE/AQY/AS0BEQEGngCIAAsB"
"BgsBKAEcAQY6ATcBDQEGDgFNATsBACYBEAEMAQBOATUBFwEASQFPAS8BCVABUQHpAAAyAVIBhgAGGwFTASQBBikBVAEfAQY0AVUBPQEA5gBWAU4BAC8BVQE0"
"AQBRARQB6QAARwE4ASABBuUAVgHmAABXAUsBOQEGSAEqAQ8BAD0BVQFIAQBYATABNQEJ6QADAVABAEQBWQE+AQZLATIBAAEGUgFMAYYABkoBWgE5AQY7AU0B"
"PAEAEAFbAeUAAFkBXAE+AQZSAV0BXgEGWgFXATkBBp4ACwEeAQZfARwBRgEGTQFgATwBACoBSAFhAQBOAWIBYwEAZAFRAVABAEUBPgFcAQYdARwBXwEGMwFl"
"AS4BBkIBOAFgAQYOAUIBYAEGDgFgAU0BBjwBQAElAQA1AWMBWAEJHgFmAVoBBp4AHgFKAQZKAR4BWgEGSwFSATIBBkUBZwFoAQZoAVMBLAEGHwFpATMBBlsB"
"agHlAABWAWIBTgEAPwFrAS0BBlIBSwFdAQZaAWwBbQEGXAFnAUUBBioBYQEEAQAwAVgBbgEJUwEbASwBBlMBbwEkAQblAGoBVgEANQFOAWMBAEMBcAFJAQkt"
"AXEBRAEGLgFyASgBBmYBHgEdAQYtAWsBcQEGOgEEAWEBAFUBcwFIAQAXAeYATgEAAwF0AVABAHEBWQFEAQZXAVoBbQEGVAF1AWkBBmABdgFAAQB3ARQBUQEA"
"CgF0AQMBACgBcgFGAQYkAW8BRwEANwE6AXgBBggBeQFBAQBBAXoBMQEAMQF0AQoBAGYBbAFaAQZHAXsBYAEGYQF8AToBAH0BYQFIAQA8AWABQAEAcwF+AUgB"
"AEABfwEmAQBuATYBMAEJQwGAAXABCUMBKwGAAQCBAXkBNgEJFAF3ASsBAGsBggFxAQZoAYMBUwEGJgF/AYQBABABJgGEAQBuAYUBNgEJCAE2AXkBAEYBhgFf"
"AQYfAVQBaQEGSAF+AX0BAGQBdwFRAQB0AYcBZAEAbAFmAR0BBjcBiAFUAQZJAXABiQEJZAGKAXcBAFABdAFkAQAuAYsBcgEGXgGCAWsBBjgBRwFgAQZ+AXMB"
"jAEHVQGNAXMBAIUBgQE2AQlZAXEBjgEGLAFFAWgBBmsBPwFeAQaPAX0BfgEAfgGMAY8BB3YBfwFAAQBWAWoBYgEATwFJAZABCUkBiQGQAQlXAV0BSwEGkQFs"
"AR0BBkYBcgGGAQYpATcBVAEGYAF7AZIBBnYBkgGTAQBgAZIBdgEAhAFbARABAGMBlAFYAQArAZUBgAEAegF0ATEBAFkBjgFcAQZfAZYBHQEGNwF4AYgBBnsB"
"lwGSAQaOAWcBXAEGXwGGAZYBBnIBmAGGAQaWAZkBkQEGewFHAW8BADoBmgGbAQZ8AZwBOgEAYQF9AXwBAH0BnQF8AQCMAZ0BjwEHLwFPAZ4BAHQBegGHAQBM"
"AVIBXgEGeAGfAYgBBp0BfQGPAQeNAZ4BoAEFVQEvAZ4BAGIBagFjAQBkAaEBigEAQQF5AYEBAKIBZQEzAQd7AW8BowEAeAGbAZ8BBqQBkwGSAQBzAY0BjAEA"
"VQGeAY0BAI4BcQGCAQZnAaUBaAEGVAGmAXUBB2kBpwEzAQdlAaIBiwEGLgFlAYsBBosBqAFyAQZ4AToBmwEGmgE6AZwBAJ0BjAGpAQeNAaABqgEAKwF3AZUB"
"AIoBlQF3AQCYAasBhgEGrAGYAXIBBq0BrAFyAQaoAa0BcgEGowGXAXsBAFsBrgFqAQCAAa8BcAEJigGhAbABAIIBsQGOAQaOAbEBZwEGsgGmAVQBB4QBrgFb"
"AQBqAZQBYwEAXgGzAYIBBrQBpQFnAQYzAacBogEHHQGWAZEBBlQBiAGyAQejAbUBlwEAtQGSAZcBAIwBjQGqAQCEAX8BtgEAcAGvAYkBCasBlgGGAQa3AbEB"
"ggEGuAG5AW8BAKYBugF1AQd1AboBaQEHjAGqAakBB08BuwGeAQCVAa8BgAEAvAGVAYoBAIoBsAG9AQBBAb4BegEAhwF6Ab8BAMABuAGDAQaxAbQBZwEGUwG4"
"AW8BBqQBkgG1AQB2AZMBfwEElAHBAW4BCb4BQQGBAQDCAWQBhwEAqwHDAZYBBpEBxAFsAQaiAacBiwEHpwGoAYsBB7MBtwGCAQafAZsBxQEHxgFPAZABAIUB"
"bgHHAQmhAcgByQEAmAGsAasBBrcBtAGxAQZdAVcBygEGxAHLAcwBBp8BxQGIAQedAc0BnAEAqQHNAZ0BAKABzQGqAQBPAcYBuwEAzgGUAWoBAFgBlAFuAQm8"
"AYoBvQEAvgG/AXoBAMMBmQGWAQZoAcABgwEGbAHMAW0BBs8BtAG3AQayAYgBxQEHkwHQAX8BBKkBqgHNAQDQAbYBfwEEngHRAaABBbsB0gGeAQXGAdIBuwEF"
"rwHTAYkBAK8BlQG8AQDUAcABaAEGUwGDAbgBBtUBzwG3AQa6AacBaQEHpwHWAagBB9cBswFeAQZvAbkBowEAmgHYAZsBBpMBpAHZAQB8AZ0BnAEA0QHNAaAB"
"AK4BhAG2AQDTAa8BvAEAhQHaAYEBANMBvAG9AQChAckBsAEA2wG+AYEBANoB2wGBAQChAcIB3AEAwgGhAWQBAIcB3QHCAQCmAd4BugEGtQGjAd8BBNgBxQGb"
"AQaaAeAB2AEG4AGaAZwBAOEB2QGkAQCuAc4BagEAaAGlAdQBBuIBsgHFAQbfAaMBuQEE3wGkAbUBAJMB2QHQAQSeAdIB0QEFyAGhAdwBANwBwgHdAQDdAYcB"
"4wEA4wGHAeQBAIcBvwHkAQCkAd8B4QEA0wG9AeUBAMcB5gGFAQCwAckByAEJ3AHnAcgBAN0B4wHnAQncAd0B5wEJ6AHpAeoBCdsB6AG+AQDkAb8B6gEAvwG+"
"AeoBAOoBvgHoAQDrAewB1AEG1AGlAesBBrIB3gGmAQayAeIB3gEG4gHFAdgBBpwB7QHgAQC2Ae4BrgEA7wGUAc4BAO8BwQGUAQluAcEBxwEJiQHTAeUBAPAB"
"8QG9AQnIAfABsAEJ6QHoAdsBCfIB8wHWAQanAfIB1gEG4AHtAdgBAM4BrgHuAQCwAfABvQEJ8AHIAecBANoB5gHbAQDkAeoB6QEAwAHUAfQBBsoB9QFdAQa4"
"AfQB9gEAugHeAacBBrkBuAH2AQDYAfcB4gEA9wHYAe0BAPYB3wG5AQT4AfkBwQEA5gHaAYUBAPoB6QHbAQnkAekB+wEA5AH7AeMBCfIBpwHeAQbNAfwBnAEA"
"7wH4AcEBAOYBxwH9AQC4AcAB9AEG4gH3Ad4BAPcB/gHeAQD/Ad8B9gEEkAGJAeUBAOUBvQEAAgDxAQECAAIAvQHxAQACAPEB8AEBAgnnAeMBAgIJAgLjAfsB"
"CT8BTAFeAQZsAcQBzAEG/AHtAZwBANIBAwLRAQXuAQQCzgEAxwHBAQUCAOcBBgIBAgD6AfsB6QEAXQEHAl4BBggC0AHZAQTRAQMCzQEABALvAc4BAJABCQLG"
"AQDnAQEC8AEABwLXAV4BBvYB9AH/AQAKAt8B/wEA7QH8AfcBAN8BCgLhAQDhAQgC2QEAtgHQAQsCBAsC7gG2AQD4Ae8BBAIACQLSAcYBBZAB5QEJAgD5AQUC"
"wQEA5gEMAtsBCQIC+wENAgnmAf0BDgIJDwIOAv0BCQICBgLnAQnbARAC+gEJxwEFAv0BAPUBBwJdAQbeAf4B8gEA4QEKAggCABEC0AEIAgQGAgICEgIADAIQ"
"AtsBCQwC5gEOAgn6ARACEwIADwL9AQUCCewB9AHUAQZtAcoBVwEGBwIUAtcBAPIB/gHzAQD+AfcBFQIAFgL/AfQBAPcBFwIVAgAXAvcB/AEACgIRAggCABgC"
"0AERAgQLAhgC7gEA5QEAAgkCAAYCEgIBAgAMAhkCGgIAEAIMAhoCAA0C+wETAgD7AfoBEwIADwIFAg4CCf4BGwLzAQAWAgoC/wEA/AHNARwCABgCCwLQAQQY"
"AgQC7gEAHQIDAtIBBQ4CGQIMAgAeAuwB6wEA9AHsAR4CABQCHwLXAQAgAh4C6wEA/AEhAhcCBfwBHAIhAgDNAQMCHAIAAwIdAhwCAB0C0gEJAgUFAvkBIgIA"
"AAIBAiMCABkCDgIiAgAiAg4CBQIAzAEkAiUCAMoBbQEmAgD1AScCFAIA9QEUAgcCABsCKALzAQAkAssBKQIAzAHLASQCAP4BFQIbAgAVAhcCKgIA9AErAhYC"
"AAoCFgIRAgAAAiMCLAIALQIuAhgCABwCHQIvAgAYAi4CBAIAMAIiAvkBADAC+QH4AQAEAjECMgIAGgIzAhACAC8CHQIJAgAJAgACLwIAEgIjAgECACUCJgJt"
"AQBtAcwBJQIAJwI0AhQCAB4CKwL0AQA1AjQCJwIANgL1ATcCADcCJgI4AgA4AjMCNwIAOQI6AjgCADUCJwI2AgAXAiECKgIAKgIhAjsCABECLQIYAgA7AiEC"
"HAIALwI8AhwCAD0CGQIiAgAuAjECBAIALwIsAjwCADACPQIiAgAwAvgBMgIALwIAAiwCACUCOAImAgDKASYCNwIA9QHKATcCACUCOQI4AgAnAvUBNgIAFAI0"
"AjUCADcCMwI+AgA+AjYCNwIANgI/AjUCADoCQAI4AgA2Aj4CGQIALQIRAhYCAD8CNgI9AgASAkACQQIAPAI7AhwCAD0CNgIZAgD4AQQCMgIAEgICAhMCAAIC"
"DQITAgAzAhMCEAIAQgIjAhICAEACEwI4AgATAjMCOAIAPgIzAhoCABkCPgIaAgBBAkICEgIAEgITAkACAEMCRAJFAgZDAkUCRgIGQwJHAkgCBkMCSAJJAgZD"
"AkkCSgIGQwJKAksCBkMCSwJMAgZDAkwCTQIGQwJNAk4CBkMCTgJEAgZPAlACUQIGTwJSAlMCBk8CUwJUAgZPAlQCVQIGTwJVAlYCBk8CVgJXAgZPAlcCWAIG"
"TwJYAlkCBk8CWQJaAgZPAloCUAIGWwJcAl0CBlsCXgJfAgZbAmACXgIGWwJhAmACBlsCYgJhAgZbAmMCYgIGWwJkAmMCBlsCZQJkAgZbAmYCZQIGWwJdAmYC"
"BmcCaAJpAgZnAmoCaAIGZwJrAmoCBmcCbAJrAgZnAm0CbAIGZwJuAm0CBmcCbwJuAgZnAnACbwIGZwJxAnACBmcCaQJyAgZzAnQCdQIGdAJ2AnUCBncCeAJ5"
"AgZ5AnoCeAIGeQJ6AnsCBnsCfAJ6AgZ9AnMCfAIGfgJ/AoACBoACfwKBAgaAAoECggIGggKBAoMCBoMChAKFAgaEAoUChgIGhgKHAogCBocCiAKJAgaHAokC"
"igIGigKJAosCBooCjAKLAgaMAosCjQIGjAKNAn4CBo4CjwKQAgaPApACkQIGjwKSApECBpICkQKTAgaSApMClAIGlAKTApUCBpQClQKWAgaWApUClwIGlgKY"
"ApcCBpgClwKZAgaYApkCmgIGmgKZApsCBpoCnAKbAgacAp0CngIGngKdAp8CBp4CnwKgAgagAp8CoQIGoAKhAqICBqICoQKjAgaiAqMCpAIGpAKjAqUCBqQC"
"pQKmAganAqYCqAIGpwKoAqkCBqkCqAKqAgapAo4CqgIGqwKsAq0CBqsCrgKtAgauAq0CrwIGrgKvArACBrACsQKvAgaxArICswIGswKyArQCBrMCtAK1Aga1"
"ArQCtgIGtQK2ArcCBrcCtgK4Aga3ArgCuQIGuQK4AroCBrkCugK7Aga7AroCvAIGuwK8Ar0CBr0CvgK8Aga9Ar4CvwIGwALBAsICBsECwgLDAgbBAsMCxAIG"
"xALDAsUCBsQCxQLGAgbGAsUCxwIGxgLIAscCBscCyQLKAgbHAssCygIGywLKAswCBssCzALNAgbNAswCwAIGzgLPAtACBtACzwLRAgbQAtEC0gIG0gLRAtMC"
"BtIC0wLUAgbVAtYC1wIG1gLXAtgCBtYC2ALZAgbZAtgC2gIG2QLbAtoCBtsC2gLcAgbbAt0C3AIG3QLcAt4CBt0CzgLeAgbfAuAC4QIG4ALiAuECBuAC4wLi"
"AgbgAuMC5AIG4ALlAuQCBuUC5gLkAgblAucC5gIG5QLnAugCBuUC6QLoAgbpAuoC6AIG6QLrAuoCBukC6wLsAgbsAusC7QIG7ALuAu0CBu4C7wLtAgbuAvAC"
"7wIG8ALxAu8CBvAC8gLxAgbwAvIC8wIG8AL0AvMCBt8C5QLgAgbfAukC5QIG3wLsAukCBt8C7gLsAgbfAvAC7gIG3wL0AvACBvUC9gL3Agb3AvYC+AIG9wL4"
"AvkCBvkC+AL6Agb5AvoC+wIG+QL8AvsCBvwC/QL7Agb8Av0C/gIG/gL9Av8CBv4C/wIAAwb+AgEDAAMG/gIBAwIDBgIDAQMDAwYCAwMDBAMGAgMEAwUDBvUC"
"9wL5Agb1AvkC/AIG9QL8Av4CBvUC/gICAwb1AgIDBQMGBgMHAwgDBgYDCQMHAwYGAwkDCgMGCgMJAwsDBgoDDAMLAwYMAw0DCwMGDAMOAw0DBgwDDgMPAwYP"
"Aw4DEAMGDwMRAxADBhEDEgMQAwYRAxMDEgMGEQMTAxQDBhQDEwMVAwYUAxYDFQMGFAMWAxcDBhcDFgMYAwYXAxkDGAMGFwMZAxoDBhoDGQMbAwYaAxwDGwMG"
"HAMdAxsDBhwDHgMdAwYcAx4DHwMGHwMeAyADBh8DIQMgAwYhAyIDIAMGBgMMAwoDBgYDDwMMAwYGAxEDDwMGBgMUAxEDBgYDFwMUAwYGAxoDFwMGBgMcAxoD"
"BgYDHwMcAwYGAyEDHwMGIwMkAyUDBiMDJQMmAwYmAyUDJwMGJgMnAygDBigDJwMpAwYoAykDKgMGKAMrAyoDBisDLAMqAwYrAywDLQMGLQMsAy4DBi4DLAMv"
"AwYuAy8DMAMGLgMwAzEDBjEDMAMyAwYxAzIDMwMGMQM0AzMDBjEDNAM1AwY1AyMDJgMGNQMmAygDBjUDKAMrAwY1AysDLQMGNQMtAy4DBjUDLgMxAwY2AzcD"
"OAMPWoM1IHfKJoRAIzw1rpFfV/rpr4IkN+KlhIISOsrAOY47TemacY2LDNH+opE+Vnz0lY9PVUAaaoJbOCE+J4MpOnBWLat9Uh06tpQPWljI741bDOo4oYqt"
"DE3GZ4wLRNuVKocgHc6dPYWTH1Bh8J3pHIB0VY9qNWxrfI/TQahfgcEEUBCL+Z99EBmMOIxmDadcxJyHVUgyaL25DJ96v5knXVS3RZw8YHceHZPNDeqi4KJr"
"IRmIF6dgQA+PXK83WNORwKMZdLTv6aG/a12+Mq6VDAhajxFAC7YxNLGKDeHDjw6tDCbrgac5c2bHyrEmciYT+7X8PpNxpac5bpCyhaL0cXgHY7HqMWF1kLRL"
"ayOfEcIKJdKAk6aEds7kgLNODjWjYdLdf//jQszCeDqdEsG6VSM3/cxuFZx9kq3YDa06z9cpC/5DofZKNwWH1czvcqqc+KswCRKuuAFGEJ6AGhj/TuBlZe9c"
"VII/UMITKS58XQDcSxCI2vCIVeOJHAeNClxV1P1IcxIXbzX/f8zvH0ITKEF+ylYlDPay1w1hbg+cKwSnOOx7qiwXc1icHFFLDrekAzGSDbd700NhV+yOqDtm"
"DruY3zweJReBxzbzFEh7KT7vPP+INkMIeYqfLW96C4hsFEHJUzQ6ZHWcDLYOq1JBDTY7OFWPDntQuFOlDAZbWlHEDTHEmnWIDa3EflkmbcwYqEN0O9tyd0GB"
"c/wSSTx4EOqGy0+Cammfcld/cVa740LCRu2KFlGVUlqO0FaaMxF0vl5EIzaI41N4GxiGuWMQHYpzz1qLEA+OBHTpQBNfK2LjT6M3X1vXdS7vcVpRDnmi1F14"
"Z5CvSHXnQ4yVwl6WbjQNCVt6cMfFsVpYdG7oenOfTdacEHJrDm1bnmcoXUi5vXE4DnGiFG+VVezzNnCrVcXpOHV5LYmUCm1mW7IgPn3dOjtW9Gs6WWLIzXKa"
"DWs4KX0ROStAd3PmVJkSCH6YOq+/03pUHaJhcnoGJECe8H41GaHK530BOdKoF318KDQ20HiHJ2RY0HjTB8JS0Hg8DKNA0HitIdU80HhZMT5HSniFJ5DBSng7"
"DKO+SnjSB4WsSniqIf+lSnhZMWqweImFJ2JYeIk6DKFAeInSB79SeImoIdE8eIlZMTtH9omrIf2l9omGJ47B9olaMWiw9ok8DKG+9onTB4Os7VgaHBNlanx1"
"LBJeTXoCHJNhDIIvMd9bIH1SNTVZ71gnO1VTOn0BO5BSX4F/PN5O7n0APQ5LYGKUPM1Gv4ABO4tDWH6lOAxA9lhTLOo2uXxGJ5k1wIOqIW41h1tKHEI2ZqkB"
"HApjQ4YCHJRhR4JgPzdPXKl6PEFREH8APQ5LSYKXPDxHSKmZOYpAlIGROBNAIn9CNfE8wYJAMTE6Wan8G08ziIUjHIA07lgEHAXMvXuqIdjJGHvrGy7K9oE7"
"J9vI/Xx1LDzHXX0vMQvF6YBCNVbC71hCNVbCsX6kODu/71jvO8m6v35/PAm4AYAAPTm071gMPT2ycn5/PGmwp4ABO7as71gJOtiqaH2ROD6pG4JCNRymZmXD"
"MYWjsGQBLAKhLoQ7J5efAGM9IpKeLGbqGx6eSKnrGy7KLYQTHNfKRKnZNHPE4IGjOYu//38BO7u754DnPBu42aiCNraisYRrJ3uddnuqIZmem6hOHICfAAAB"
"AAIAAAMABAAFAAAAAAIABAAAAQAAAAYAAAcAAgABAAAIAAEACQAACQAKAAsAAAEACAAHAAAEAAIADAAABgANAAEAAAYAAAAOAAADAA8AEAAAEQASABMAAAoA"
"EQATAAATABQACgAABQAPAAMAABUADwAFAAUPABYAEAAACgAUAAsAABEAFwASAAAJABgACAAAFwAZABIAAAwAGgAEAAAIABsABwAJEAAWABwAAA8AHQAWAAAe"
"AB0ADwAAGgAFAAQAAB4ADwAVAAULABgACQAABQAfABUAABsACAAYAAAHACAAAgAAIQAaAAwAACIAGQAXAAYGACMADQAGJAAGAA4ABgYAJAAlAAYMACYAIQAA"
"AgAmAAwACRgAJwAbAAAUABMAKAAEKQAaACEACSAAGwAqAAkgAAcAGwAJEgArABMAAAUAKQAsAAAFABoAKQAAJwAqABsAAB0AHgAtAAAFACwAHwAAAgAuACYA"
"CQIAIAAuAAAcABYALwAGJgAuADAAACgAEwArAAQhACYAMQAAMAAxACYAADAALgAgAAAtABYAHQAGCwAUACgAADIAGAALAAksACkAMQAAKQAhADEAACoAMAAg"
"AAASABkAMwAAEgAzACsAACIANAA1AAYeADYALQAAMQA3ACwAAC8AJAA4AAYwACoAJwAALQA5ABYABhUANgAeAAALADoAOwAAKwAzADwABhUAPQA2AAALADsA"
"MgAADQAjADQABiwANwAfAAk3AD4AHwAJNQA/ACIABkAAMAAnAAAiAD8AGQAGIwA1ADQABh8APgA9AAAyAEAAGAAJQABBADAAADIAOwBAAAlAACcAGAAJQgA8"
"ADMABhUAHwA9AAA6AAsAKAAAPAAoACsAACQAQwAlAAY3AEQAPgAJRQAoADwAAC8AQwAkAAZGADcAMQAALwA5AEcABhkASAAzAAA9AD4ASQAAQgBFADwAACgA"
"RQA6AAAGACUAIwAGOQBKAEcABhYAOQAvAAZJAD4ARAAJLQBLADkABkQANwBGAAkvAEcAQwAGSABMADMAAD0ATQA2AABOAEYAMQAAIwA/ADUABkEATgAxAAAZ"
"AE8ASAAGOgBQADsAAFAAQAA7AAkxADAAQQAAIwBRAFIABjYATQBLAAA/AFMAVAAGVABPAD8ABiMAJQBRAAYlAFUAVgAGNgBLAC0AAEAAUABXAAlPABkAPwAG"
"UgA/ACMABkIAWABFAABFAFgAOgAAQQBAAFkAADkAWgBKAAZVACUAQwAGRABGAFsACUEAXABOAAAzAEwAQgAGPQBdAE0AAFcAWQBAAAlJAEQAXgAJRABbAF4A"
"CVgAQgBfAABSAFMAPwAGSwBgAGEABjkAYQBaAAY9AEkAXQAATABIAGIAAFoAYwBKAAc5AEsAYQAGYgBCAEwAADoAWABkAABPAGIASAAASQBeAF0AAE4AWwBG"
"AABfAEIAYgAAUABlAFcACWYAQQBZAABKAGcARwAHTgBcAGgAAGMAWgBhAAdfAGQAWAAEXgBpAF0ABWMAZwBKAAdpAE0AXQAAWwBOAGgAAGoAZgBZAABXAGoA"
"WQAAXABBAGsAAGsAQQBsAABbAGgAbQAAbABBAGYAAG4ATwBUAAZjAGEAYAAGUAA6AGQAAFwAbwBoAAlvAFwAawAAYABwAGMAAHEAZgBqAAlsAGYAcgAAcABn"
"AGMABk0AcwBLAABqAFcAdAAAXgBbAG0AAGgAbwBtAABrAGwAcgAJcwBgAEsAAGQAdQBQAABXAGUAdAAAawB2AG8AAHEAcgBmAABRAHcAUgAGdQBlAFAAAGAA"
"cwBwAAB4AGUAdQAAbQBpAF4ABXgAdABlAABqAHQAeQAJcgB2AGsACXYAcgB6AAB5AHEAagAJVgBRACUABnsAYgBPAABwAHMAfAAAYgBkAF8AAHYAegBvAAB7"
"AE8AbgAATQBpAHMAAHQAeAB5AABtAG8AegAAUQBWAH0AAGIAewBkAABtAHoAfgAAfwBRAH0AAGkAfgBzAAB4AH8AeQAAaQBtAH4AAHcAUQB/AAB5AH8AfQAA"
"dQBkAHsAAH4AfABzAAB5AHIAcQAAegByAH0AAHIAeQB9AACAAIEAggAGgACCAIMABoAAgwCEAAaFAIYAhwAGhQCHAIgABoUAiACJAAaKAIsAjAAGigCNAIsA"
"BooAjgCNAAaPAJAAkQAGjwCSAJAABo8AkwCSAAaUAJUAlgAGlACXAJUABpQAlwCYAAaUAJkAmAAGmQCaAJgABpkAmwCaAAaZAJsAnAAGmQCdAJwABp0AngCf"
"AAadAKAAnwAGoAChAJ8ABqAAogChAAagAKIAowAGlACdAJkABpQAoACdAAaUAKMAoAAGpAClAKYABqQApgCnAAanAKYAqAAGpwCpAKgABqcAqQCqAAaqAKkA"
"qwAGqgCrAKwABqoArQCsAAaqAK0ArgAGrgCtAK8ABqQApwCqAAakAKoArgAGsACxALIABrAAswCxAAawALMAtAAGsAC1ALQABrAAtgC1AAawALYAtwAGtwC2"
"ALgABrcAuQC4AAa5ALoAuAAGuQC7ALoABrkAuwC8AAa8ALsAvQAGvAC+AL0ABrwAvgC/AAa/AL4AwAAGvwDBAMAABr8AwQDCAAbDAMQAxQAGsAC5ALcABrAA"
"vAC5AAawAL8AvAAGsADCAL8ABrAAwwDCAAawAMUAwwAGsADGAMUABscAyADJAAbJAMgAygAGyQDKAMsABskAzADLAAbJAMwAzQAGzQDMAM4ABs0AzgDPAAbN"
"ANAAzwAG0ADHAMkABtAAyQDNAAY="
;
static const char* CARDATA_SEDAN2 =
"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"
"AQZOAVYBRAEJIwHuAEIBADYBLwFIAQBLAVcBWAEAUAErAUYBAEgBQgHuAAUvAVgBSAEAKQFZAVIBAFcBSwE0AQBEAVYBWgEJBQE/ATgBADkBPgFFAQBVAVsB"
"PwEAQQE9AS4BBkkBUgFKAQk8AVwBTAEAQwFdAVEBBkIBXgFdAQBSAVkBSgEJQwFCAV0BAF8BKQFHAQAGAUQBNAEJUAFaAU8BAEIBYAFeAQBYAWEBSAEAXwFZ"
"ASkBAFUBOgFFAQZhAUIBSAEFTwFaAVYBAD0BYgE+AQYvAUsBWAEAVwFjAVgBAEQBVwE0AQlFAWQBVQEGSgFZAWUBCTgBPwFmAQBYAWcBYQEAWgFoAUQBCT8B"
"WwFmAQBpAWABQgEAQgFhAWkBBWQBagFbAQZmAWsBOAEATQFsAUYBAG0BTQFMAQBkAW4BagEAXAFvAUwBAEYBbAFQAQBVAWQBWwEGYwFnAVgBAHABPgFiAQZx"
"AWgBWgEJPQFyAWIBBlkBcwF0AQB1AWMBVwEJaAFXAUQBCRgBdgFBAQZwAW4BZAEAawFHATgBAHcBWQFfAQA8AUoBZQEJZQFcATwBCUUBPgFkAQBgAWkBeAEA"
"YQF4AWkBBXkBYAF4AQBZAXQBegEJbwFtAUwBAG4BewFbAQZ3AXMBWQEAYAF5AV4BAGUBbwFcAQlkAT4BcAEAagFuAVsBBnoBdAFzAQl6AWUBWQEJVwFoAXwB"
"CX0BbAFNAQB+AVEBXQEGRwFrAV8BAGcBfwFhAQCAAXEBWgEAUAFsAYABAIEBTQFtAQCCAWIBcgEAgwFmAVsBAGsBdwFfAQBvAYQBbQEAhQFnAWMBAGMBdQGF"
"AQCAAYYBcQEAGAEXAQgBBl4BeQFdAQCGAYcBcQEJgAFaAVABAIgBYgGCAQCIAXABYgEGcAF7AW4BAHsBiQFbAQZhAYoBeAEFcwF3AYsBAHMBiwF6AQlNAYEB"
"fQEAQQGCAXIBBj0BQQFyAQZRAX4BOwEGYQF/AYoBBYQBbwFlAQmEAYEBbQEAgQGMAX0BAGwBfQGNAQB+AV0BjgEGjwGEAWUBCZABkQGCAQZwAYgBkgEGXQF5"
"AY4BAIMBWwGJAQZ6AZMBZQEJaAFxAYcBCZQBlQGWAQaWAXYBGAEGOwF+AVQBBmcBlwF/AQA3AZgBmQEGmQGaATcBBjcBmgGbAQabAQsBNwEGCwGcAQwBBpwB"
"DgEMAQaWARgBCAEGlQF2AZYBBnYBkAFBAQZBAZABggEGeAGdAXkBAHgBigGdAQWeAZ8BdwEAawGeAXcBAJcBigF/AQV3AZ8BiwEAZwGFAZcBAHUBVwF8AQls"
"AaABgAEANwGhAZgBBqEBogGYAQaIAaMBkgEAkgGkAXsBAGYBpQFrAQCPAYEBhAEApgGGAYABAI4BpwF+AQakAagBewEAZgGDAakBAIUBdQGqAQCrAaoBdQEA"
"pgGsAYYBAKABpgGAAQCNAaABbAEArQGNAX0BAK0BfQGMAQCjAa4BkgEAkgGuAaQBAHkBrwGOAQCJAXsBqAEGegGLAbABCbEBdQF8AQmyAbMBjwEAjwFlAbIB"
"AKgBtAGJAQaJAbQBqQEEqQGDAYkBBJcBtQGKAQV1AbEBqwEJkwGyAWUBCYcBtgF8AQCHAXwBaAEAjwC3AZAABrgBuQGQAQZ+AacBVAEGpwG6AVQBBqMBiAGC"
"AQBwAZIBewEAnQGvAXkBAIsBnwGwAQCxAbsBvAEJsQF8AbsBCbIBkwG9AQCNAa0BvgEJjAG/AcABCcEBvwGMAQnAAa0BjAEAjAGBAcEBAFQBugE3AQaKAbUB"
"nQEFlwGFAaoBALEBvAGrAQB8AcIBwwEAsgG9AbMBAKwBhwGGAQCgAcQBpgEAgQGPAcUBAMUBxgHBAQnBAYEBxQEAjgGvAacBBscBawGlAQDHAZ4BawEAnwHI"
"AbABAKsBvAGqAQDIAZMBegEArAGmAcQBAK0BwAG/AQDFAY8BswEAowHJAa4BAJ0BygHLAQC1AcoBnQEFzAGfAZ4BAMgBegGwAQC2AYcBrAEAswHNAc4BCc8B"
"0AGsAQDEAc8BrAEAxgG/AcEBCcUBswHOAQm/AdEBrQEA0gGpAbQBBGYBqQGlAQDHAdMBngEAtQGXAaoBBbsBfAHDAQC2AdQBwgEJxQHOAcYBCbYBwgF8AQA3"
"AboB1QEG1gGjAYIBANcBrwHLAQCoAdIBtAEArwGdAcsBAM4B2AHGAQnNAbMBvQEJ1AG2AawBCdEBvgGtAQnQAdkBrAEJzwHaAdABANsB3AHEAQnbAcQBoAEJ"
"NwHdAaEBBpEB1gGCAQDdATcB1QEGpwGvAdcBBskB3gGkAQauAckBpAEGuwHfAbwBAMgB4AHhAQDDAcIB1AEA2QHUAawBCdoB2QHQAQDGAeIBvwEA4wG9AeEB"
"Cb0BkwHhAQDJAaMB1gEAqAGkAeQBAM0B5QHOAQnUAdkB2gEA2AHmAcYBCZMByAHhAQDPAcQB3AEJ2gHPAdwBAOIB0QG/AQC9AeMBzQEJ4QHnAeMBCegB6QG3"
"AQaRAZABuQEG5AHqAagBANcBywHrAQDLAcoB6wEAygHsAesBAMoB7QHsAQXtAcoBtQEF7gHIAZ8BALwB3wHvAQDwAdgBzgEJ8AHOAeUBCeYB4gHGAQDbAaAB"
"vgEJoAGNAb4BCeIBvgHRAQmnAfEBugEG8gHzAfQBBvUB9gH3AQbyAfgB8wEG+QH6AfsBBskB/AHeAQb9Af4B/wEG+gEAAvsBBvgBAQLzAQYCAvYB9QEGpwHX"
"AfEBBvoBAwIAAgABAvgBBAIA9gECAgUCAAMCBgIAAgDeAeQBpAEGBwL/Af4BBggCCQIHAgD/AQcCCQIGBAIKAgsCAAwCDQIOAgAGAgwCDgIACgIPAgsCAKgB"
"6gHSAQAPAhACCwIApQGpAdIBBO0B6wHsAQDtAe8B6wEAtQHvAe0BBbUBqgHvAQURAsgB7gEA5gHYAfABANwB2wHaAQDlAc0BEgIJ4AHnAeEBCRMC6QHoAQAU"
"AZsACAEGtwHpAZAABhQCFQIIAQaRAbkBFgIA9wEXAvUBBvcBGAIXAgYZAvIB9AEGGgL9ARsCBgMCDAIGAgAbAv0B/wEGHAL2AQUCAPgBCgIEAgAJAh0C/wEG"
"HgIKAvgBAOoBHwLSAQAcAgUCIAIAHAIgAiECAA0CIgIOAgAJAggCIgIACQIiAiMCABwCIQIQAgAiAg0CIwIAEAIhAgsCANMBzAGeAQDMAe4BnwEAJALwAeUB"
"AN8BuwHDAQDUAdoBwwEACAElAhQCBrkBJgIWAgAnAhoCGwIG/AHJAdYBACgC+AHyAQbXASkC8QEAKgLqAd4BBt4B6gHkAQYpAtcB6wEAHwIrAtIBACwCyAER"
"AgAsAuAByAEAzQEtAhICACwC5wHgAQDdAS4CoQEAoQEvAjACBhMCMQLpAQCQACUCCAEGFAIyAhUCAJABMwK4AQYxAjQCGAIAFwIYAjQCAPkB+wE1Agb8AdYB"
"FgIA9gEYAvcBBjYC+gH5AQYYAvYBNwIG9gEcAjgCAKUB0gErAgTHAaUBOQIExwE5AtMBBKoBvAHvAQDlARICLQIAogGhATACBrgBMwI6AgA6AiYCuAEAuAEm"
"ArkBANYBkQEWAgA1AjsC+QEGugHxATwCBtUBugE8AgYoAvIBGQIGNgL5ATsCBt4B/AEWAgD/AR0CGwIGPALxASkCAPoBPQI+AgDqASoCHwIAPwI4AhwCAAwC"
"AwI+AgBAAh0CCQIGDAI+Ag0CAEACCQJBAgAeAkICDwIACgIeAg8CABACQgI/AgAJAiMCQQIADwJCAhACAA0CQwJBAgANAkECIwIA6wHvAUQCAN8BwwFFAgDm"
"AfABJAIAJAJGAuYBAOMBLQLNAQBHAt0B1QEGRwIuAt0BAC4CLwKhAQAxAkgC6QEA6QFIAkkCAOkBSQKQAACQAEoCJQIASgJLAiUCACUCMgIUAgD0AUwCGQIA"
"TAJNAhkCABkCTQIuAgAbAh0CJwIGMQIYAjcCADYCPQL6AQY3AvYBOAIAAwL6AT4CAPgBKAJOAgAeAvgBTgIAPgJDAg0CABwCEAI/AgA5AisC0wEATwIRAu4B"
"AOYBRgLiAQAkAuUBUAIAUALlAS0CAFEC4wHnAQA1AksCOwIA1QE8AkcCBjcCUgJTAgAWAioC3gEAUgI3AlQCACkC6wFEAgArAjkCpQEELAIRAk8CAL4B2gHb"
"AQDDAdoBVQIAUAItAlYCACwCVgJXAgAxAlMCSAIASQJYApAAAJAAWQJKAgBZAloCSgIASwIyAiUCACYCOgJbAgBIAlMCSQIAWAJZApAAAFwCGQIuAgAxAjcC"
"UwIAJgIqAhYCAFoCXQI7AgAoAhkCXAIGWAJeAlkCAEkCUwJfAgBYAkkCXwIAKQJOAlwCAFgCXwJGAgBgAj4CPQIAVAJhAmICAD4CYAJjAgBkAmUCYwIA0wFk"
"AswBACQCUAJeAgDfAUUCRAIAwwFVAkUCAFACLAJPAgBQAlYCLAIAVwLnASwCAOMBUQJmAgAtAuMBZgIAOwJKAloCAEsCSgI7AgAuAkcCXAIAWwJnAh8CAB8C"
"KgImAgBHAjwCXAIAXAJOAigCADwCKQJcAgA2Al0CPQIAJgJbAh8CAF0CNgI7AgBdAmACPQIARAJOAikCAFICYgJTAgBSAlQCYgIAXgJaAlkCAGICXwJTAgBf"
"AmIC2gEAZwJoAisCAB4CTgJFAgBoAtMBKwIAHwJnAisCAGACXQJPAgBgAk8CZAIAZAJjAmACANMBaAJpAgBqAlUCawIAaQJkAtMBAEYCXgJYAgBeAkYCJAIA"
"XQJaAl4CAGQCaQJlAgC+AV8C2gEA7wHfAUQCAF4CUAJdAgDuAWQCTwIAvgFGAl8CAF0CUAJPAgBVAtoBYgIARgK+AeIBAGYCVgItAgBEAkUCTgIAYgJhAlUC"
"AGECawJVAgBFAlUCagIAZALuAcwBAOcBVwJRAgBWAmYCVwIAVwJmAlECAGwCbQJuAgZsAm4CbwIGbAJvAnACBmwCcAJxAgZsAnECcgIGbAJzAnQCBmwCdAJ1"
"AgZsAnUCdgIGbAJ2AncCBmwCdwJtAgZ4AnkCegIGeAJ6AnsCBngCewJ8AgZ4AnwCfQIGeAJ9An4CBngCfgJ/AgZ4An8CgAIGeAKAAoECBngCgQKCAgZ4AoIC"
"gwIGhAKFAoYCBoQChwKFAgaEAogChwIGhAKJAogCBoQCigKJAgaEAosCigIGhAKMAosCBoQCjQKMAgaEAo4CjwIGhAKGAo4CBpACkQKSAgaQApMCkQIGkAKU"
"ApUCBpAClgKUAgaQApcClgIGkAKYApcCBpACmQKYAgaQApoCmQIGkAKbApoCBpACkgKbAgacAp0CngIGngKfAp0CBqACoQKiAgahAqMCogIGoQKjAqQCBqQC"
"pQKjAgalAqYCowIGpwKoAqkCBqoCqQKrAgaqAqsCrAIGrAKtAq4CBq4CrQKvAgauAq8CsAIGsAKvArECBrACsQKyAgayArECswIGsgK0ArMCBrUCtgK3Aga4"
"ArkCtgIGuAK5AroCBroCuwK5Aga6ArwCuwIGvAK9ArsCBrwCvgK9Aga+Ar8CvQIGvgK/AsACBsACvwLBAgbBAsICvwIGwgLDAsQCBsICxQLDAgbFAsYCwwIG"
"xQLHAsYCBscCxgLIAgbIAskCxgIGyALKAskCBsoCywLJAgbMAs0CzgIGzALPAs0CBs8CzQLQAgbQAtEC0gIG0QLSAtMCBtEC0wLUAgbUAtMC1QIG1ALWAtUC"
"BtYC1QLXAgbWAtgC1wIG2QLaAtsCBtoC3ALbAgbcAtsC3QIG3ALeAt0CBt8C3QLgAgbfAuEC4AIG4QLgAuICBuEC4gLjAgbjAuIC5AIG4wLlAuQCBuUC5ALM"
"AgbmAucC6AIG5gLoAukCBukC6ALqAgbqAusC7AIG7ALtAu4CBu0C7gLvAgbtAu8C8AIG8ALxAvICBvAC8gLzAgbzAvIC9AIG8wL1AvQCBvUC9AL2Agb1AvYC"
"9wIG9wL2AvgCBvcC+AL5Agb5AvgC+gIG+QL7AvoCBvsC5gL6Agb8Av0C/gIG/gL/AgADBv8CAAMBAwb/AgEDAgMGAgMDAwQDBgQDBQMDAwYEAwUDBgMGBgMF"
"AwcDBggDCQMKAwYIAwkDCwMGCAMMAwsDBgwDDQMLAwYMAw4DDQMGDgMPAw0DBg4DEAMPAwYOAxADEQMGEQMQAxIDBhEDEwMSAwYTAxQDEgMGFAMVAxIDBhQD"
"FgMVAwYUAxYDFwMGFwMWAxgDBhcDGQMYAwYXAxoDGQMGGgMbAxkDBggDDgMMAwYIAxEDDgMGCAMTAxEDBggDFAMTAwYIAxcDFAMGCAMaAxcDBhwDHQMeAwYe"
"Ax0DHwMGHgMfAyADBh4DIQMgAwYhAyIDIAMGIQMiAyMDBiEDJAMjAwYkAyUDIwMGJAMlAyYDBiYDJQMnAwYmAycDKAMGJgMpAygDBikDKgMoAwYpAyoDKwMG"
"KQMrAywDBiwDKwMtAwYsAy0DLgMGLgMtAy8DBi4DLwMwAwYuAzADMQMGLgMxAzIDBjIDHAMeAwYyAx4DIQMGMgMhAyQDBjIDJAMmAwYyAyYDKQMGMgMpAywD"
"BjIDLAMuAwYzAzQDNQMGMwM2AzQDBjYDNwM0AwY2AzgDNwMGNgM5AzgDBjkDOgM4AwY6AzsDOAMGOgM8AzsDBjwDPQM7AwY8Az4DPQMGPAM/Az4DBj8DQAM+"
"AwY/A0EDQAMGPwNCA0EDBkIDNgMzAwZCAzkDNgMGQgM6AzkDBkIDPAM6AwZCAz8DPAMGQwNEA0UDBkMDRANGAwZGA0QDRwMGRgNHA0gDBkYDSANJAwZJA0gD"
"SgMGSQNKA0sDBkkDSwNMAwZMA0sDTQMGTANNA04DBkwDTgNPAwZPA04DUAMGTwNQA1EDBlEDUANSAwZRA0MDRgMGUQNGA0kDBlEDSQNMAwZRA0wDTwMGUwNU"
"A1UDD2WVEEjTWF+mrVVtMjSLpTssRueM7hQGnoaTXhIYqg+OPSN3pLuRvg/+X5uUdhzcbIqKDCV5X6+MejZZq9iL8jb0ZHmK3jjWVzKORzdABDSbhlji7fmL"
"VSCbzVyWo1Q7IiqOgCkuNzmKKTclxoiUCFfNGw6H0jxZueuRRg5kyB6SFBD73C6R1BNHPA+TwC90bLKX5D5zbcKVRBA8Pq2bB1cS/GaaLCYpjcmY8ylEPa+Q"
"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"
"AEgAPwAASABKAD8AAEUAPgAiAAk+AEUASwAJMABAAEkABkEAQwBMAAZBAEwARAAASwBHAD4AACUATQBCAAZAAE4ATwAGSQBAAE8ABkoARQA/AABQAFEANwAJ"
"NwA1AFAACUoASAA3AABJAE8AQwAARwAxAD0ABTgAUgAlAAZEAFAANQAATQAoAEIABjcAUQBKAAAlAFIATQAGOAAxAFMAAEcAUwAxAAU4AFQAUgAGOABTAFQA"
"AFUAUwBHAAVDAFYATAAGUQBXAEoAAEsAVQBHAABYAEsARQAJQABZAE4ABlEAUABXAAlDAE8AVgAAWgBXAFAACVsARABMAABcAFUASwAAQAAtAEYABkoAXQBe"
"AABXAF0ASgAASgBeAF8AAFkAYABOAAZbAEwAVgAGRgBZAEAABk0AYQAoAAZTAGIAVAAAUwBVAGIABWMAWgBQAABEAGMAUAAAVQBcAGQAAFwASwBYAAlKAGUA"
"RQAATQBmAGEABmYAZwBhAAZPAGgAVgAAZQBYAEUAAEQAWwBpAABlAGoAWAAAXwBlAEoAAGkAWwBWAARkAGIAVQAFawBgAFkABmgATwBOAABcAFgAbAAJVwBa"
"AG0AAFgAbgBvAABXAG0AXQAAXQBwAF4ACWMARABpAABcAGwAZAAAXwBeAHAAAF0AbQBwAAllAHEAagAAcAByAF8AAHMAaQBWAARsAFgAbwAAagBvAG4ACWoA"
"bgBYAABNAFIAZgAGdABoAE4AAFQAYgB1AABgAHQATgAAUgBUAHUABnQAdgBoAAZxAG8AagAAVgBoAHYAAG0AdwBwAAlaAHgAbQAAeQB6ACgABnEAZQByAAll"
"AF8AcgAJewB8AGcABn0AfgB5AAZWAHYAcwAAKAB6ACkABn8AfgB9AAB7AIAAfAAAgQCCAIMAAIQAggCBAACFAH8AgAAAYwB4AFoAAGQAbABvAABgAIYAdAAA"
"awCGAGAAAFIAdQBmAAaHAIgAKQAGggCEAIkABoUAfgB/AACDAIcAgQAAZABvAIoAAHcAiwBwAACDAIgAhwAAcACLAHIAAHUAYgBkAABzAGMAaQAEbwBxAIoA"
"AHoAiwApAAB5AH4AegAAhgB2AHQAAIsAhwApAAB3AG0AeAAAZwBmAHsAAGYAdQB7AABkAHsAdQAAfgCLAHoAAIsAfgBxAACGAIwAcwAAgAB7AIoAAIwAYwBz"
"AAB2AIYAcwAAjAB4AGMAAIcAiwB3AAByAIsAcQAAhwB3AHgAAIoAcQB+AABkAIoAewAAjQCOAI8ABo0AjwCQAAaNAJEAkgAGjQCSAJMABo0AkwCOAAaUAJUA"
"lgAGlACWAJcABpQAlwCYAAaUAJgAmQAGlACZAJoABpsAnACdAAabAJ4AnAAGmwCfAJ4ABpsAoACfAAabAJ0AoQAGogCjAKQABqIApQCmAAaiAKcApQAGogCo"
"AKcABqIApACoAAapAKoAqwAGrACtAK4ABqwArwCuAAavALAArgAGrwCxALAABq8AsQCyAAavALMAsgAGswC0ALIABrMAtQC0AAazALUAtgAGtgC3ALgABqwA"
"swCvAAasALYAswAGrAC3ALYABrkAugC7AAa5ALsAvAAGuQC9ALwABr0AvgC8AAa9AL4AvwAGvQDAAL8ABsAAwQC/AAbAAMEAwgAGwgDBAMMABsIAwwDEAAbC"
"AMUAxAAGxQDGAMQABsUAxgDHAAbHAMYAyAAGxwC5AL0ABscAvQDAAAbHAMAAwgAGxwDCAMUABskAygDLAAbJAMwAygAGzADNAMoABswAzgDNAAbOAM8AzQAG"
"zgDMAMkABtAA0QDSAAbQANEA0wAG0wDRANQABtMA1ADVAAbVANQA1gAG1QDWANcABtcA0ADTAAbXANMA1QAG"
;
static const CarModelDesc CAR_MODELS_SRC[] = {
  {"СЕДАН", CARDATA_SEDAN, 822,1300,171,300, {1.0455f,1.4531f,2.4546f}, {0.784f,0.331f,1.468f,0.331f,0.225f}, {0.784f,0.331f,-1.366f,0.331f,0.225f}, 2.444f,-2.455f, {0.752f,0.760f,1.896f}, {0.677f,0.846f,-2.188f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,8,13,23},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,20,20,24},{14,51,51,56},{15,224,227,235}}},
  {"ХЭТЧБЕК", CARDATA_HATCH, 867,1300,193,299, {0.9335f,1.5373f,1.8639f}, {0.722f,0.314f,1.158f,0.314f,0.176f}, {0.720f,0.318f,-1.275f,0.318f,0.180f}, 1.859f,-1.864f, {0.614f,0.831f,1.422f}, {0.601f,0.694f,-1.708f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"УНИВЕРСАЛ", CARDATA_WAGON, 869,1300,195,300, {1.0069f,1.4707f,2.3950f}, {0.785f,0.339f,1.539f,0.339f,0.299f}, {0.786f,0.343f,-1.319f,0.341f,0.306f}, 2.395f,-2.393f, {0.661f,0.673f,2.114f}, {0.711f,0.831f,-2.107f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"КРОССОВЕР", CARDATA_CROSS, 831,1300,195,300, {1.1290f,1.7606f,2.4765f}, {0.851f,0.374f,1.578f,0.374f,0.288f}, {0.851f,0.374f,-1.389f,0.374f,0.288f}, 2.463f,-2.468f, {0.472f,0.710f,2.317f}, {0.727f,1.140f,-2.152f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"ВНЕДОРОЖНИК", CARDATA_BIGSUV, 1050,1300,262,299, {1.1334f,1.9024f,2.4824f}, {0.810f,0.406f,1.536f,0.406f,0.272f}, {0.810f,0.406f,-1.308f,0.406f,0.272f}, 2.482f,-2.469f, {0.736f,0.976f,2.108f}, {0.697f,1.097f,-2.298f}, 12, {{0,236,236,240},{1,26,36,46},{2,26,26,28},{3,235,235,245},{4,143,148,135},{5,204,13,13},{6,20,20,23},{7,153,153,161},{10,51,54,61},{16,26,46,128},{12,107,110,117},{15,224,227,235}}},
  {"МИКРОАВТОБУС", CARDATA_MINIVAN, 948,1300,235,300, {1.0853f,1.8564f,2.4723f}, {0.793f,0.328f,1.565f,0.328f,0.244f}, {0.793f,0.328f,-1.435f,0.328f,0.244f}, 2.464f,-2.467f, {0.688f,0.882f,2.186f}, {0.660f,1.252f,-2.294f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"СТАРЫЙ ДЖИП", CARDATA_OLDJEEP, 785,1299,190,299, {1.0876f,1.9527f,2.3082f}, {0.868f,0.403f,1.458f,0.403f,0.266f}, {0.868f,0.446f,-1.202f,0.446f,0.266f}, 2.308f,-2.304f, {0.770f,0.936f,2.133f}, {0.964f,0.982f,-2.124f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"ПИКАП", CARDATA_PICKUP, 789,1300,191,300, {1.1212f,2.0768f,2.9999f}, {0.890f,0.471f,2.010f,0.533f,0.400f}, {0.870f,0.490f,-1.651f,0.549f,0.440f}, 2.947f,-2.942f, {0.826f,0.966f,2.295f}, {0.950f,1.136f,-2.647f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"МОЩНЫЙ СЕДАН", CARDATA_SPORTSEDAN, 976,1299,230,299, {1.0273f,1.4089f,2.4090f}, {0.790f,0.348f,1.548f,0.348f,0.326f}, {0.820f,0.344f,-1.309f,0.344f,0.288f}, 2.399f,-2.377f, {0.582f,0.526f,1.959f}, {0.335f,0.332f,-2.291f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"СТАРЫЙ СЕДАН", CARDATA_OLDSEDAN, 808,1299,211,300, {0.9882f,1.4479f,2.3855f}, {0.721f,0.323f,1.576f,0.323f,0.316f}, {0.718f,0.323f,-1.245f,0.322f,0.329f}, 2.386f,-2.373f, {0.626f,0.628f,2.235f}, {0.627f,0.780f,-2.272f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"СПОРТКАР", CARDATA_SPORT1, 939,1300,213,300, {0.9888f,1.2722f,2.2524f}, {0.746f,0.337f,1.299f,0.336f,0.312f}, {0.743f,0.335f,-1.240f,0.332f,0.308f}, 2.251f,-2.252f, {0.581f,0.580f,1.989f}, {0.447f,0.778f,-2.050f}, 13, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,168,173,161},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,8,13,23},{10,36,38,46},{11,204,209,217},{12,107,110,117}}},
  {"СУПЕРКАР", CARDATA_SPORT2, 776,1299,193,300, {1.0750f,1.1209f,2.2524f}, {0.737f,0.329f,1.110f,0.338f,0.364f}, {0.830f,0.335f,-1.341f,0.334f,0.404f}, 2.047f,-2.245f, {0.677f,0.444f,1.854f}, {0.657f,0.553f,-2.204f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"ХЭТЧБЕК", CARDATA_HATCH2, 825,1299,209,299, {0.9036f,1.4876f,2.2412f}, {0.762f,0.324f,1.314f,0.324f,0.226f}, {0.736f,0.324f,-1.327f,0.324f,0.271f}, 2.241f,-2.235f, {0.658f,0.573f,1.951f}, {0.636f,0.696f,-1.989f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
  {"СЕДАН", CARDATA_SEDAN2, 854,1300,216,299, {1.0000f,1.5645f,2.4735f}, {0.814f,0.352f,1.472f,0.352f,0.236f}, {0.814f,0.352f,-1.360f,0.352f,0.236f}, 2.455f,-2.455f, {0.819f,0.750f,2.037f}, {0.444f,0.954f,-2.324f}, 16, {{0,236,236,240},{1,26,36,46},{2,15,15,18},{3,204,209,219},{4,224,224,189},{5,204,13,13},{6,20,20,23},{7,153,153,161},{8,242,128,13},{9,28,38,54},{10,36,38,46},{11,204,209,217},{12,107,110,117},{13,13,13,15},{14,97,99,107},{15,224,227,235}}},
};

struct MeshTri { uint16_t a, b, c; uint8_t mat; uint8_t flags; V3 n; V3 cen; };
enum { TF_STRIPE = 1 };
struct Mesh { std::vector<V3> v; std::vector<MeshTri> t; };
struct WheelDef { V3 p; float r, w; bool steer; };

struct CarModel {
    std::string name;
    Mesh lod[2];
    std::vector<MatEntry> mats;
    std::vector<WheelDef> wheels;
    float halfW = 0.9f, height = 1.4f, halfL = 2.3f, minZ = -2.3f, maxZ = 2.3f;
    V3 plateF, plateR; float plateScale = 1;
    V3 headL, tailL;           // левая фара/фонарь (x>0 — зеркалим)
    V3 exhaust;
    float roofY = 1.4f, doorX = 0.9f, doorZ = 0.2f, doorY = 0.6f;
    float wheelbase = 2.6f;
    float dpsY = 0.8f; bool procStripe = false; float stripeY = 0.55f;
    std::vector<float> sideZ, sideX;   // профиль борта для полосы ливреи
    float trunkZ = -2.0f, trunkY = 0.9f, trunkW = 0.6f;   // задний проём
    bool tallRear = false;     // открывается дверь 5-го типа
    float mass = 1.0f;
    int hitchModel = -1; float hitchZ = 0, kingpinZ = 0;   // фура
    float radius = 2.5f;
};
static std::vector<CarModel> CM;

static void finalizeMesh(Mesh& m) {
    for (size_t i = 0; i < m.t.size(); i++) {
        MeshTri& t = m.t[i];
        V3 a = m.v[t.a], b = m.v[t.b], c = m.v[t.c];
        t.n = norm(cross(b - a, c - a));
        t.cen = (a + b + c) * (1.0f / 3);
    }
}

// ---- base64 ----
static std::vector<uint8_t> b64decode(const char* s) {
    static int8_t T[256]; static bool init = false;
    if (!init) { for (int i = 0; i < 256; i++) T[i] = -1; const char* a = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"; for (int i = 0; i < 64; i++) T[(uint8_t)a[i]] = (int8_t)i; init = true; }
    std::vector<uint8_t> out; uint32_t acc = 0; int bits = 0;
    for (const char* p = s; *p; p++) {
        int v = T[(uint8_t)*p]; if (v < 0) continue;
        acc = (acc << 6) | (uint32_t)v; bits += 6;
        if (bits >= 8) { bits -= 8; out.push_back((uint8_t)((acc >> bits) & 255)); }
    }
    return out;
}

// профиль борта: максимальная ширина кузова на высоте полосы вдоль длины
static void buildSideProfile(CarModel& M, bool flat = false) {
    M.sideZ.clear(); M.sideX.clear();
    float z0 = M.minZ + 0.35f, z1 = M.maxZ - 0.35f;
    if (M.wheels.size() >= 4) { z0 = M.wheels[2].p.z + M.wheels[2].r + 0.08f; z1 = M.wheels[0].p.z - M.wheels[0].r - 0.08f; }
    int n = std::max(2, (int)((z1 - z0) / 0.25f));
    for (int i = 0; i <= n; i++) {
        float z = z0 + (z1 - z0) * i / n, best = 0;
        for (const V3& v : M.lod[0].v) if (fabsf(v.z - z) < 0.18f && fabsf(v.y - M.stripeY) < 0.12f) best = std::max(best, fabsf(v.x));
        if (flat) best = M.doorX; else if (best < 0.3f) continue;   // процедурные: плоский борт
        M.sideZ.push_back(z); M.sideX.push_back(best);
    }
    float dx = 0;
    if (!flat) for (const V3& v : M.lod[0].v) if (fabsf(v.z - M.doorZ) < 0.45f && fabsf(v.y - M.dpsY) < 0.1f) dx = std::max(dx, fabsf(v.x));
    if (dx > 0.3f) M.doorX = dx;
}
static void loadEmbeddedModel(const CarModelDesc& d) {
    CarModel M;
    M.name = d.name;
    std::vector<uint8_t> raw = b64decode(d.data);
    size_t off = 0;
    int nvs[2] = {d.nv0, d.nv1}, nts[2] = {d.nt0, d.nt1};
    for (int L = 0; L < 2; L++) {
        Mesh& m = M.lod[L];
        m.v.resize(nvs[L]);
        for (int i = 0; i < nvs[L]; i++) {
            int16_t q[3]; memcpy(q, &raw[off], 6); off += 6;
            m.v[i] = V3(q[0] / 32767.0f * d.ext[0], q[1] / 32767.0f * d.ext[1], q[2] / 32767.0f * d.ext[2]);
        }
        m.t.resize(nts[L]);
        for (int i = 0; i < nts[L]; i++) {
            uint16_t ix[3]; memcpy(ix, &raw[off], 6); off += 6;
            MeshTri& t = m.t[i]; t.a = ix[0]; t.b = ix[1]; t.c = ix[2]; t.mat = raw[off++]; t.flags = 0;
        }
        finalizeMesh(m);
    }
    for (int i = 0; i < d.nm; i++) M.mats.push_back(d.mats[i]);
    // габариты
    float mnx = 1e9f, mxx = -1e9f, mxy = 0, mnz = 1e9f, mxz = -1e9f;
    for (const V3& v : M.lod[0].v) { mnx = std::min(mnx, v.x); mxx = std::max(mxx, v.x); mxy = std::max(mxy, v.y); mnz = std::min(mnz, v.z); mxz = std::max(mxz, v.z); }
    M.halfW = (mxx - mnx) * 0.5f; M.height = mxy; M.minZ = mnz; M.maxZ = mxz; M.halfL = (mxz - mnz) * 0.5f;
    M.wheels.push_back({V3(d.fw[0], d.fw[1], d.fw[2]), d.fw[3], d.fw[4], true});
    M.wheels.push_back({V3(-d.fw[0], d.fw[1], d.fw[2]), d.fw[3], d.fw[4], true});
    M.wheels.push_back({V3(d.rw[0], d.rw[1], d.rw[2]), d.rw[3], d.rw[4], false});
    M.wheels.push_back({V3(-d.rw[0], d.rw[1], d.rw[2]), d.rw[3], d.rw[4], false});
    M.wheelbase = d.fw[2] - d.rw[2];
    M.headL = V3(d.hl[0], d.hl[1], d.hl[2]); M.tailL = V3(d.tl[0], d.tl[1], d.tl[2]);
    // номер — по высоте бампера
    M.plateF = V3(0, 0.48f, d.pf + 0.025f); M.plateR = V3(0, 0.55f, d.pr - 0.025f);
    // крыша, двери, задний проём
    float roof = 0; float door = 0; float trunkTop = 0;
    for (const V3& v : M.lod[0].v) {
        if (fabsf(v.x) < 0.3f && fabsf(v.z) < 0.6f) roof = std::max(roof, v.y);
        if (v.z > -0.3f && v.z < 0.7f && v.y > 0.45f && v.y < 0.75f) door = std::max(door, fabsf(v.x));
        if (fabsf(v.x) < 0.3f && v.z < M.minZ + 0.6f) trunkTop = std::max(trunkTop, v.y);
    }
    M.roofY = roof > 0.5f ? roof : M.height; M.doorX = door > 0.3f ? door : M.halfW; M.doorZ = 0.2f; M.doorY = 0.62f;
    M.trunkZ = M.minZ + 0.15f; M.trunkY = trunkTop > 0.5f ? trunkTop : 0.95f; M.trunkW = M.halfW * 0.72f;
    M.tallRear = (M.trunkY > 1.15f);
    M.dpsY = M.height * 0.56f;
    M.exhaust = V3(-M.halfW * 0.55f, 0.28f, M.minZ + 0.05f);
    M.radius = sqrtf(M.halfW * M.halfW + M.halfL * M.halfL);
    M.stripeY = M.height * 0.4f;
    M.procStripe = true;
    buildSideProfile(M);
    CM.push_back(M);
}

// ---------------------------------------------------------------------
//  Процедурные модели
// ---------------------------------------------------------------------
struct MeshB {
    Mesh* m; V3 center;
    int addV(const V3& p) { m->v.push_back(p); return (int)m->v.size() - 1; }
    // треугольник с проверкой ориентации (нормаль от центра)
    void tri(const V3& a, const V3& b, const V3& c, int mat, V3 ref) {
        int ia = addV(a), ib = addV(b), ic = addV(c);
        V3 n = cross(b - a, c - a); V3 cc = (a + b + c) * (1.f / 3);
        MeshTri t; t.mat = (uint8_t)mat; t.flags = 0;
        if (dot(n, cc - ref) < 0) { t.a = (uint16_t)ia; t.b = (uint16_t)ic; t.c = (uint16_t)ib; }
        else { t.a = (uint16_t)ia; t.b = (uint16_t)ib; t.c = (uint16_t)ic; }
        m->t.push_back(t);
    }
    void quad(const V3& a, const V3& b, const V3& c, const V3& d, int mat, V3 ref) { tri(a, b, c, mat, ref); tri(a, c, d, mat, ref); }
    // параллелепипед
    void box(V3 c, V3 h, int mat, bool bottom = false, int matTop = -1) {
        V3 p[8];
        for (int i = 0; i < 8; i++) p[i] = V3(c.x + ((i & 1) ? h.x : -h.x), c.y + ((i & 2) ? h.y : -h.y), c.z + ((i & 4) ? h.z : -h.z));
        quad(p[1], p[3], p[7], p[5], mat, c);  // +x
        quad(p[0], p[4], p[6], p[2], mat, c);  // -x
        quad(p[2], p[6], p[7], p[3], matTop >= 0 ? matTop : mat, c);  // +y
        if (bottom) quad(p[0], p[1], p[5], p[4], mat, c);
        quad(p[4], p[5], p[7], p[6], mat, c);  // +z
        quad(p[0], p[2], p[3], p[1], mat, c);  // -z
    }
    // плоская наклейка (прямоугольник в плоскости), смотрит по normal
    void decal(V3 c, V3 ax, V3 ay, int mat, V3 normal) { quad(c - ax - ay, c + ax - ay, c + ax + ay, c - ax + ay, mat, c - normal); }
};

struct Sec { float z, yb, yBelt, yTop, wB, wT; int cab; };
// «лофт» кузова из поперечных сечений
static void loft(MeshB& B, const std::vector<Sec>& S, int paint, int glass, int black) {
    const int NP = 12;
    auto ring = [](const Sec& s, V3* p) {
        float r = std::min(0.09f, (s.yTop - s.yBelt) * 0.4f);
        p[0] = V3(0, s.yb, s.z); p[1] = V3(s.wB * 0.9f, s.yb, s.z); p[2] = V3(s.wB, s.yb + 0.1f, s.z); p[3] = V3(s.wB, s.yBelt, s.z);
        p[4] = V3(s.wT, s.yTop - r, s.z); p[5] = V3(s.wT * 0.8f, s.yTop, s.z); p[6] = V3(0, s.yTop, s.z);
        p[7] = V3(-s.wT * 0.8f, s.yTop, s.z); p[8] = V3(-s.wT, s.yTop - r, s.z); p[9] = V3(-s.wB, s.yBelt, s.z); p[10] = V3(-s.wB, s.yb + 0.1f, s.z); p[11] = V3(-s.wB * 0.9f, s.yb, s.z);
    };
    for (size_t i = 0; i + 1 < S.size(); i++) {
        V3 a[NP], b[NP]; ring(S[i], a); ring(S[i + 1], b);
        const Sec &s0 = S[i], &s1 = S[i + 1];
        for (int k = 0; k < NP; k++) {
            int k1 = (k + 1) % NP; int mat = paint;
            if (k == 0 || k == 11) mat = black;
            else if (k == 3 || k == 8) mat = (s0.cab && s1.cab) ? glass : (s0.cab != s1.cab ? glass : paint);
            else if (k >= 4 && k <= 7) mat = (s0.cab != s1.cab) ? glass : paint;
            V3 ref = V3(0, (s0.yb + s0.yTop) * 0.5f, (s0.z + s1.z) * 0.5f);
            if (dist2d(a[k], b[k]) < 1e-4f && fabsf(a[k].y - b[k].y) < 1e-4f && dist2d(a[k1], b[k1]) < 1e-4f && fabsf(a[k1].y - b[k1].y) < 1e-4f) continue;
            B.quad(a[k], a[k1], b[k1], b[k], mat, ref);
        }
    }
    // торцы
    for (int e = 0; e < 2; e++) {
        const Sec& s = e == 0 ? S.front() : S.back();
        V3 p[NP]; ring(s, p);
        V3 c(0, (s.yb + s.yTop) * 0.5f, s.z);
        V3 ref = c + V3(0, 0, e == 0 ? 1.f : -1.f);
        for (int k = 0; k < NP; k++) B.tri(c, p[k], p[(k + 1) % NP], paint, ref);
    }
}

static int addMat(CarModel& M, int type, int r, int g, int b) {
    MatEntry e; e.type = (uint8_t)type; e.r = (uint8_t)r; e.g = (uint8_t)g; e.b = (uint8_t)b;
    M.mats.push_back(e); return (int)M.mats.size() - 1;
}
struct StdMats { int paint, glass, black, chrome, head, tail, ind, paint2, frame; };
static StdMats stdMats(CarModel& M) {
    StdMats s;
    s.paint = addMat(M, M_PAINT, 236, 236, 240);
    s.glass = addMat(M, M_GLASS, 26, 36, 46);
    s.black = addMat(M, M_BLACK, 22, 22, 25);
    s.chrome = addMat(M, M_CHROME, 190, 192, 198);
    s.head = addMat(M, M_HEAD, 225, 225, 200);
    s.tail = addMat(M, M_TAIL, 200, 15, 15);
    s.ind = addMat(M, M_IND, 242, 128, 13);
    s.paint2 = addMat(M, M_PAINT2, 60, 90, 160);
    s.frame = addMat(M, M_FRAME, 50, 50, 55);
    return s;
}
static void finishProcModel(CarModel& M) {
    finalizeMesh(M.lod[0]);
    M.lod[1] = M.lod[0];
    float mnx = 1e9f, mxx = -1e9f, mxy = 0, mnz = 1e9f, mxz = -1e9f;
    for (const V3& v : M.lod[0].v) { mnx = std::min(mnx, v.x); mxx = std::max(mxx, v.x); mxy = std::max(mxy, v.y); mnz = std::min(mnz, v.z); mxz = std::max(mxz, v.z); }
    M.halfW = (mxx - mnx) * 0.5f; M.height = mxy; M.minZ = mnz; M.maxZ = mxz; M.halfL = (mxz - mnz) * 0.5f;
    M.radius = sqrtf(M.halfW * M.halfW + M.halfL * M.halfL);
    M.procStripe = true;
    M.lod[1] = M.lod[0];
    buildSideProfile(M, true);
}

// ВАЗ-2107 «Жигули»
static void buildClassic() {
    CarModel M; M.name = "ЖИГУЛИ";
    StdMats s = stdMats(M);
    MeshB B; B.m = &M.lod[0];
    std::vector<Sec> S = {
        {-2.06f, 0.32f, 0.78f, 0.84f, 0.74f, 0.70f, 0}, {-1.98f, 0.27f, 0.82f, 0.92f, 0.80f, 0.77f, 0},
        {-1.12f, 0.26f, 0.84f, 0.95f, 0.81f, 0.78f, 0}, {-0.82f, 0.26f, 0.86f, 1.40f, 0.81f, 0.63f, 1},
        {0.52f, 0.26f, 0.86f, 1.42f, 0.81f, 0.64f, 1}, {1.02f, 0.26f, 0.84f, 0.95f, 0.81f, 0.78f, 0},
        {1.97f, 0.28f, 0.78f, 0.86f, 0.80f, 0.76f, 0}, {2.07f, 0.33f, 0.70f, 0.78f, 0.74f, 0.70f, 0}};
    loft(B, S, s.paint, s.glass, s.black);
    V3 nf(0, 0, 1), nr(0, 0, -1);
    B.decal(V3(0, 0.6f, 2.085f), V3(0.36f, 0, 0), V3(0, 0.08f, 0), s.chrome, nf);       // решётка
    for (int k = -1; k <= 1; k += 2) {
        B.decal(V3(k * 0.56f, 0.6f, 2.086f), V3(0.15f, 0, 0), V3(0, 0.075f, 0), s.head, nf);
        B.decal(V3(k * 0.58f, 0.62f, -2.076f), V3(0.16f, 0, 0), V3(0, 0.085f, 0), s.tail, nr);
        B.decal(V3(k * 0.39f, 0.62f, -2.076f), V3(0.04f, 0, 0), V3(0, 0.085f, 0), s.ind, nr);
        B.box(V3(k * 0.86f, 0.98f, 0.95f), V3(0.06f, 0.05f, 0.04f), s.black, true);    // зеркала
    }
    B.box(V3(0, 0.40f, 2.12f), V3(0.80f, 0.06f, 0.05f), s.chrome, true);   // бамперы
    B.box(V3(0, 0.40f, -2.11f), V3(0.80f, 0.06f, 0.05f), s.chrome, true);
    B.box(V3(0, 0.62f, 0), V3(0.815f, 0.012f, 1.9f), s.chrome);             // молдинг
    M.wheels = {{V3(0.66f, 0.30f, 1.26f), 0.30f, 0.18f, true}, {V3(-0.66f, 0.30f, 1.26f), 0.30f, 0.18f, true},
                {V3(0.66f, 0.30f, -1.16f), 0.30f, 0.18f, false}, {V3(-0.66f, 0.30f, -1.16f), 0.30f, 0.18f, false}};
    M.wheelbase = 2.42f;
    M.plateF = V3(0, 0.40f, 2.18f); M.plateR = V3(0, 0.5f, -2.09f);
    M.headL = V3(0.56f, 0.6f, 2.09f); M.tailL = V3(0.58f, 0.62f, -2.08f);
    M.exhaust = V3(-0.45f, 0.25f, -2.1f);
    M.roofY = 1.42f; M.doorX = 0.815f; M.doorZ = 0.15f; M.doorY = 0.6f; M.dpsY = 0.74f; M.stripeY = 0.5f;
    M.trunkZ = -1.9f; M.trunkY = 0.93f; M.trunkW = 0.62f;
    finishProcModel(M);
    CM.push_back(M);
}

// «Газель» — маршрутное такси
static void buildMinibus() {
    CarModel M; M.name = "МАРШРУТКА";
    StdMats s = stdMats(M);
    MeshB B; B.m = &M.lod[0];
    std::vector<Sec> S = {
        {-2.80f, 0.48f, 1.25f, 2.18f, 0.97f, 0.94f, 1}, {-2.72f, 0.42f, 1.25f, 2.28f, 1.0f, 0.97f, 1},
        {1.42f, 0.42f, 1.25f, 2.28f, 1.0f, 0.97f, 1}, {2.02f, 0.42f, 1.20f, 1.26f, 1.0f, 0.95f, 0},
        {2.62f, 0.44f, 1.00f, 1.08f, 0.97f, 0.90f, 0}, {2.80f, 0.50f, 0.86f, 0.95f, 0.90f, 0.82f, 0}};
    loft(B, S, s.paint, s.glass, s.black);
    V3 nf(0, 0, 1), nr(0, 0, -1);
    B.decal(V3(0, 0.72f, 2.815f), V3(0.42f, 0, 0), V3(0, 0.1f, 0), s.black, nf);
    for (int k = -1; k <= 1; k += 2) {
        B.decal(V3(k * 0.66f, 0.74f, 2.812f), V3(0.16f, 0, 0), V3(0, 0.08f, 0), s.head, nf);
        B.decal(V3(k * 0.8f, 0.9f, -2.815f), V3(0.08f, 0, 0), V3(0, 0.16f, 0), s.tail, nr);
        B.box(V3(k * 1.08f, 1.55f, 1.95f), V3(0.05f, 0.12f, 0.05f), s.black, true);
    }
    B.decal(V3(0, 1.7f, -2.815f), V3(0.8f, 0, 0), V3(0, 0.35f, 0), s.glass, nr);    // заднее стекло
    B.box(V3(0, 0.5f, 2.85f), V3(0.95f, 0.08f, 0.06f), s.black, true);
    B.box(V3(0, 0.5f, -2.85f), V3(0.95f, 0.08f, 0.06f), s.black, true);
    // табличка маршрута
    B.box(V3(0, 2.0f, 1.43f), V3(0.35f, 0.07f, 0.01f), s.ind, true);
    M.wheels = {{V3(0.82f, 0.36f, 2.0f), 0.36f, 0.22f, true}, {V3(-0.82f, 0.36f, 2.0f), 0.36f, 0.22f, true},
                {V3(0.82f, 0.36f, -1.45f), 0.36f, 0.24f, false}, {V3(-0.82f, 0.36f, -1.45f), 0.36f, 0.24f, false}};
    M.wheelbase = 3.45f;
    M.plateF = V3(0, 0.62f, 2.82f); M.plateR = V3(0, 0.7f, -2.83f);
    M.headL = V3(0.66f, 0.74f, 2.83f); M.tailL = V3(0.8f, 0.9f, -2.83f);
    M.exhaust = V3(-0.6f, 0.3f, -2.7f);
    M.roofY = 2.28f; M.doorX = 1.0f; M.doorZ = 0.0f; M.doorY = 0.9f;
    M.trunkZ = -2.7f; M.trunkY = 2.0f; M.trunkW = 0.8f; M.tallRear = true;
    M.mass = 1.6f;
    finishProcModel(M);
    CM.push_back(M);
}

// кабина грузовика (общая для грузовика и тягача)
static void truckCab(MeshB& B, StdMats& s, float z0, float z1, float yb, float top, float w) {
    std::vector<Sec> S = {
        {z0, yb, 1.75f, top, w, w - 0.08f, 1}, {z1 - 0.18f, yb, 1.75f, top, w, w - 0.08f, 1},
        {z1, yb, 1.70f, top - 0.18f, w - 0.02f, w - 0.14f, 0}};
    loft(B, S, s.paint, s.glass, s.black);
    V3 nf(0, 0, 1);
    B.decal(V3(0, 1.25f, z1 + 0.012f), V3(w * 0.55f, 0, 0), V3(0, 0.22f, 0), s.black, nf);
    for (int k = -1; k <= 1; k += 2) {
        B.decal(V3(k * (w - 0.25f), 0.95f, z1 + 0.012f), V3(0.16f, 0, 0), V3(0, 0.09f, 0), s.head, nf);
        B.box(V3(k * (w + 0.12f), 2.0f, z1 - 0.3f), V3(0.04f, 0.22f, 0.06f), s.black, true);
    }
    B.box(V3(0, 0.75f, z1 + 0.05f), V3(w, 0.13f, 0.09f), s.black, true);
}

static void buildTruck() {
    CarModel M; M.name = "ГРУЗОВИК";
    StdMats s = stdMats(M);
    MeshB B; B.m = &M.lod[0];
    truckCab(B, s, 1.95f, 3.55f, 0.85f, 2.85f, 1.2f);
    B.box(V3(0, 0.68f, -0.1f), V3(0.48f, 0.13f, 3.65f), s.black, true);              // рама
    B.box(V3(0, 2.15f, -1.05f), V3(1.25f, 1.2f, 2.85f), s.paint2, false);           // будка
    for (int i = 0; i < 6; i++) {                                                     // рёбра будки
        float z = -3.8f + i * 1.13f;
        for (int k = -1; k <= 1; k += 2) B.box(V3(k * 1.255f, 2.15f, z), V3(0.012f, 1.2f, 0.05f), s.frame);
    }
    B.box(V3(0, 0.95f, -3.95f), V3(1.2f, 0.08f, 0.04f), s.black, true);
    V3 nr(0, 0, -1);
    for (int k = -1; k <= 1; k += 2) {
        B.decal(V3(k * 0.95f, 0.95f, -3.995f), V3(0.13f, 0, 0), V3(0, 0.06f, 0), s.tail, nr);
        B.box(V3(k * 1.1f, 0.85f, -0.9f), V3(0.1f, 0.12f, 0.9f), s.frame, true);     // боковые ящики
    }
    M.wheels = {{V3(1.0f, 0.5f, 2.7f), 0.5f, 0.3f, true}, {V3(-1.0f, 0.5f, 2.7f), 0.5f, 0.3f, true},
                {V3(0.98f, 0.5f, -1.55f), 0.5f, 0.42f, false}, {V3(-0.98f, 0.5f, -1.55f), 0.5f, 0.42f, false},
                {V3(0.98f, 0.5f, -2.75f), 0.5f, 0.42f, false}, {V3(-0.98f, 0.5f, -2.75f), 0.5f, 0.42f, false}};
    M.wheelbase = 4.8f;
    M.plateF = V3(0, 0.75f, 3.66f); M.plateR = V3(0, 0.8f, -4.0f);
    M.headL = V3(0.95f, 0.95f, 3.57f); M.tailL = V3(0.95f, 0.95f, -4.0f);
    M.exhaust = V3(1.15f, 2.9f, 1.85f);
    M.roofY = 3.35f; M.doorX = 1.2f; M.doorZ = 2.7f; M.doorY = 1.6f;
    M.trunkZ = -3.9f; M.trunkY = 3.2f; M.trunkW = 1.15f; M.tallRear = true;
    M.mass = 4;
    finishProcModel(M);
    CM.push_back(M);
}

static void buildTractor() {
    CarModel M; M.name = "ФУРА";
    StdMats s = stdMats(M);
    MeshB B; B.m = &M.lod[0];
    truckCab(B, s, 1.0f, 3.0f, 0.9f, 3.45f, 1.25f);
    B.box(V3(0, 0.72f, 0.0f), V3(0.5f, 0.15f, 3.05f), s.black, true);
    B.box(V3(0, 0.92f, -1.6f), V3(0.6f, 0.06f, 0.6f), s.frame, true);    // седло
    B.box(V3(0, 3.55f, 1.6f), V3(1.1f, 0.12f, 0.6f), s.paint, true);      // обтекатель
    for (int k = -1; k <= 1; k += 2) {
        B.box(V3(k * 1.1f, 0.85f, 0.3f), V3(0.18f, 0.28f, 0.55f), s.chrome, true);   // баки
        B.box(V3(k * 0.9f, 2.6f, 0.85f), V3(0.07f, 1.0f, 0.07f), s.chrome);           // трубы
    }
    M.wheels = {{V3(1.02f, 0.52f, 2.15f), 0.52f, 0.32f, true}, {V3(-1.02f, 0.52f, 2.15f), 0.52f, 0.32f, true},
                {V3(1.0f, 0.52f, -1.2f), 0.52f, 0.45f, false}, {V3(-1.0f, 0.52f, -1.2f), 0.52f, 0.45f, false},
                {V3(1.0f, 0.52f, -2.45f), 0.52f, 0.45f, false}, {V3(-1.0f, 0.52f, -2.45f), 0.52f, 0.45f, false}};
    M.wheelbase = 4.0f;
    M.plateF = V3(0, 0.75f, 3.16f); M.plateR = V3(0, 0.75f, -3.08f);
    M.headL = V3(1.0f, 0.95f, 3.02f); M.tailL = V3(0.9f, 0.8f, -3.06f);
    M.exhaust = V3(0.9f, 3.65f, 0.85f);
    M.roofY = 3.67f; M.doorX = 1.25f; M.doorZ = 2.0f; M.doorY = 1.7f;
    M.hitchModel = (int)CM.size() + 1; M.hitchZ = -1.6f;
    M.mass = 6;
    finishProcModel(M);
    CM.push_back(M);
}
static void buildTrailer() {
    CarModel M; M.name = "ПОЛУПРИЦЕП";
    StdMats s = stdMats(M);
    MeshB B; B.m = &M.lod[0];
    B.box(V3(0, 2.65f, 0), V3(1.27f, 1.38f, 6.8f), s.paint2, true);
    B.box(V3(0, 1.15f, -1.0f), V3(0.45f, 0.12f, 5.6f), s.black, true);
    for (int i = 0; i < 12; i++) for (int k = -1; k <= 1; k += 2) B.box(V3(k * 1.275f, 2.65f, -6.5f + i * 1.18f), V3(0.012f, 1.38f, 0.04f), s.frame);
    V3 nr(0, 0, -1);
    for (int k = -1; k <= 1; k += 2) {
        B.decal(V3(k * 0.95f, 1.05f, -6.83f), V3(0.15f, 0, 0), V3(0, 0.07f, 0), s.tail, nr);
        B.box(V3(k * 0.7f, 0.7f, 3.0f), V3(0.05f, 0.5f, 0.05f), s.frame, true);   // опоры
        B.box(V3(k * 1.0f, 0.9f, 0.5f), V3(0.02f, 0.28f, 2.6f), s.frame);          // боковая защита
    }
    B.box(V3(0, 0.82f, -6.85f), V3(1.25f, 0.08f, 0.04f), s.black, true);
    M.wheels = {{V3(1.0f, 0.5f, -3.9f), 0.5f, 0.45f, false}, {V3(-1.0f, 0.5f, -3.9f), 0.5f, 0.45f, false},
                {V3(1.0f, 0.5f, -5.0f), 0.5f, 0.45f, false}, {V3(-1.0f, 0.5f, -5.0f), 0.5f, 0.45f, false},
                {V3(1.0f, 0.5f, -6.1f), 0.5f, 0.45f, false}, {V3(-1.0f, 0.5f, -6.1f), 0.5f, 0.45f, false}};
    M.plateF = V3(0, -5, 0); M.plateR = V3(0, 1.05f, -6.86f);
    M.headL = V3(0, -5, 0); M.tailL = V3(0.95f, 1.05f, -6.86f);
    M.exhaust = V3(0, -5, 0);
    M.roofY = 4.03f; M.doorX = 1.27f;
    M.trunkZ = -6.7f; M.trunkY = 3.8f; M.trunkW = 1.2f; M.tallRear = true;
    M.kingpinZ = 5.7f;
    M.mass = 8;
    finishProcModel(M);
    CM.push_back(M);
}

// единичное колесо: ось по X, наружная сторона +X
static Mesh WHEEL[2];
static void buildWheelMesh() {
    for (int L = 0; L < 2; L++) {
        Mesh& m = WHEEL[L];
        int N = L == 0 ? 16 : 8;
        // материалы: 0 шина, 1 диск, 2 тёмное (ступица/провалы диска)
        auto addT = [&](V3 a, V3 b, V3 c, int mat, V3 ref) {
            int i = (int)m.v.size(); m.v.push_back(a); m.v.push_back(b); m.v.push_back(c);
            MeshTri t; t.mat = (uint8_t)mat; t.flags = 0;
            if (dot(cross(b - a, c - a), (a + b + c) * (1.f / 3) - ref) < 0) { t.a = (uint16_t)i; t.b = (uint16_t)(i + 2); t.c = (uint16_t)(i + 1); }
            else { t.a = (uint16_t)i; t.b = (uint16_t)(i + 1); t.c = (uint16_t)(i + 2); }
            m.t.push_back(t);
        };
        for (int i = 0; i < N; i++) {
            float a0 = i * 2 * PI / N, a1 = (i + 1) * 2 * PI / N;
            float c0 = cosf(a0), s0 = sinf(a0), c1 = cosf(a1), s1 = sinf(a1);
            V3 o(0, 0, 0);
            // протектор
            addT(V3(-0.5f, s0, c0), V3(0.5f, s0, c0), V3(0.5f, s1, c1), 0, V3(0, 0, 0));
            addT(V3(-0.5f, s0, c0), V3(0.5f, s1, c1), V3(-0.5f, s1, c1), 0, V3(0, 0, 0));
            // боковины (внешняя: кольцо шины, затем диск)
            float rr = 0.66f;
            addT(V3(0.5f, s0, c0), V3(0.5f, s0 * rr, c0 * rr), V3(0.5f, s1 * rr, c1 * rr), 0, V3(-1, 0, 0));
            addT(V3(0.5f, s0, c0), V3(0.5f, s1 * rr, c1 * rr), V3(0.5f, s1, c1), 0, V3(-1, 0, 0));
            bool spoke = L == 1 || (i % 3 != 2);
            addT(V3(0.46f, s0 * rr, c0 * rr), V3(0.46f, s1 * rr, c1 * rr), V3(0.5f, 0, 0), spoke ? 1 : 2, V3(-1, 0, 0));
            // внутренняя сторона
            addT(V3(-0.5f, s0, c0), V3(-0.5f, s1, c1), V3(-0.5f, 0, 0), 2, V3(1, 0, 0));
            (void)o;
        }
        finalizeMesh(m);
    }
}


// ---------------------------------------------------------------------
//  Отрисовка машины
// ---------------------------------------------------------------------
struct DrawCar {
    V3 pos; float yaw = 0, pitch = 0, roll = 0;
    Col paint = Col(0.9f, 0.9f, 0.9f), paint2 = Col(0.2f, 0.35f, 0.7f);
    int plate = -1;
    bool police = false, taxi = false, flash = false, headOn = false, brake = false, reverse = false;
    int blink = 0;            // -1 левый, 1 правый, 2 аварийка
    float spin = 0, steer = 0, time = 0;
    float trunk = 0;          // 0..1 открыт багажник
    bool hideBody = false;    // вид из кабины
    bool highlight = false;   // подсветка цели
    float camDist = 0;
};

struct Mat3 { V3 c0, c1, c2; };  // столбцы
static inline V3 mul(const Mat3& m, const V3& v) { return m.c0 * v.x + m.c1 * v.y + m.c2 * v.z; }
static Mat3 rotYPR(float yaw, float pitch, float roll) {
    // сначала крен (вокруг z), затем тангаж (вокруг x), затем курс (вокруг y)
    float cy = cosf(yaw), sy = sinf(yaw), cp = cosf(pitch), sp = sinf(pitch), cr = cosf(roll), sr = sinf(roll);
    auto apply = [&](V3 v) {
        V3 a(v.x * cr - v.y * sr, v.x * sr + v.y * cr, v.z);
        V3 b(a.x, a.y * cp - a.z * sp, a.y * sp + a.z * cp);
        return V3(b.x * cy + b.z * sy, b.y, -b.x * sy + b.z * cy);
    };
    Mat3 m; m.c0 = apply(V3(1, 0, 0)); m.c1 = apply(V3(0, 1, 0)); m.c2 = apply(V3(0, 0, 1)); return m;
}

static std::vector<V3> gTmpV;
static int CAR_CULL = 1;    // направление отсечения задних граней (проверено тестом)

static Col matColor(const CarModel& M, const DrawCar& d, const MeshTri& t, float& emis, int& special) {
    const MatEntry& me = M.mats[t.mat];
    Col base(me.r / 255.f, me.g / 255.f, me.b / 255.f);
    emis = 0; special = 0;
    switch (me.type) {
        case M_PAINT:
            base = d.paint;
            break;
        case M_PAINT2: base = d.paint2; break;
        case M_GLASS: case M_TINT: special = 1; break;
        case M_HEAD:
            if (d.headOn) { base = Col(1, 0.97f, 0.85f); emis = 1; }
            break;
        case M_TAIL:
            if (d.brake) { base = Col(1, 0.1f, 0.08f); emis = 1; }
            else if (d.headOn) { base = Col(0.75f, 0.05f, 0.04f); emis = 0.8f; }
            if (d.reverse && t.cen.y < M.tailL.y && fabsf(t.cen.x) < M.tailL.x) { base = Col(1, 1, 1); emis = 1; }
            break;
        case M_IND: {
            bool on = false;
            if (d.blink != 0 && fmodf(d.time, 0.8f) < 0.4f) {
                if (d.blink == 2) on = true;
                else if (d.blink == 1 && t.cen.x > 0) on = true;   // правый поворот — правая сторона (+x)
                else if (d.blink == -1 && t.cen.x < 0) on = true;
            }
            if (on) { base = Col(1, 0.6f, 0.1f); emis = 1; }
        } break;
        case M_CHROME: case M_BADGEC: case M_RIM: special = 2; break;
        default: break;
    }
    return base;
}

static inline bool me_isPaint(const CarModel& M, const MeshTri& t) { uint8_t ty = M.mats[t.mat].type; return ty == M_PAINT || ty == M_PAINT2; }
// отрисовать сетку с матрицей; возвращает false если вне поля зрения
static void drawMesh(const CarModel& M, const Mesh& m, const Mat3& rot, const V3& pos, const DrawCar& d, float bias) {
    // матрица мир->вид
    const V3 &Rr = R3.R, &Uu = R3.U, &Ff = R3.F;
    V3 rc0(dot(rot.c0, Rr), dot(rot.c0, Uu), dot(rot.c0, Ff));
    V3 rc1(dot(rot.c1, Rr), dot(rot.c1, Uu), dot(rot.c1, Ff));
    V3 rc2(dot(rot.c2, Rr), dot(rot.c2, Uu), dot(rot.c2, Ff));
    V3 tp = R3.toView(pos);
    if (gTmpV.size() < m.v.size()) gTmpV.resize(m.v.size());
    for (size_t i = 0; i < m.v.size(); i++) { const V3& p = m.v[i]; gTmpV[i] = rc0 * p.x + rc1 * p.y + rc2 * p.z + tp; }
    V3 camW = R3.pos;
    for (size_t i = 0; i < m.t.size(); i++) {
        const MeshTri& t = m.t[i];
        float emis; int special;
        Col base = matColor(M, d, t, emis, special);
        V3 nw = mul(rot, t.n);
        V3 cw = mul(rot, t.cen) + pos;
        Col c;
        if (special == 1) {
            // стекло: тёмное + отражение неба по Френелю + бегущий блик
            V3 vd = norm(cw - camW);
            float ndv = fabsf(dot(nw, vd));
            float fr = 0.18f + 0.75f * powf(1 - ndv, 2.0f);
            float streak = 0.5f + 0.5f * sinf((cw.x + cw.z) * 0.45f + cw.y * 2.0f + nw.x * 2);
            streak = streak * streak * streak * streak;
            Col sky = lerpc(L.skyHor, L.skyTop, clampf(nw.y, 0, 1));
            c = base * (0.6f + 0.4f * (1 - L.night)) + sky * (fr * (0.65f - L.night * 0.4f)) + Col(1, 1, 1) * (streak * 0.22f * (1 - L.night * 0.7f) * (0.3f + fr));
        } else if (special == 2) {
            V3 vd = norm(cw - camW);
            V3 rf = vd - nw * (2 * dot(vd, nw));
            float sp = clampf(dot(rf, L.sun), 0, 1); sp = powf(sp, 8);
            c = shadeN(base, nw, cw, emis) + Col(1, 1, 1) * (sp * 0.6f * (1 - L.night));
        } else {
            c = shadeN(base, nw, cw, emis);
            if (me_isPaint(M, t)) {
                // лёгкий лаковый блик на краске
                V3 vd = norm(cw - camW);
                V3 rf = vd - nw * (2 * dot(vd, nw));
                float sp = clampf(dot(rf, L.sun), 0, 1); sp = sp * sp; sp = sp * sp; sp = sp * sp;
                c = c + Col(1, 1, 1) * (sp * 0.35f * (1 - L.night));
            }
        }
        if (d.highlight) c = c + Col(0.12f, 0.1f, 0.0f) * (0.5f + 0.5f * sinf(d.time * 8));
        float fz = (gTmpV[t.a].z + gTmpV[t.b].z + gTmpV[t.c].z) * (1.f / 3);
        float ff = fogFactor(fz);
        if (emis > 0.5f) ff *= 0.5f;
        c = lerpc(c, L.fog, ff);
        SDL_Color sc = toSDL(c);
        R3.triView(gTmpV[t.a], gTmpV[t.b], gTmpV[t.c], sc, sc, sc, UV_WHITE, UV_WHITE, UV_WHITE, 2, CAR_CULL, bias);
    }
}

static void drawWheel(const WheelDef& w, const Mat3& bodyRot, const V3& pos, float yaw, const DrawCar& d, int lod, bool leftSide) {
    const Mesh& m = WHEEL[lod];
    float st = w.steer ? d.steer : 0;
    float wy = yaw + st;
    // матрица колеса: масштаб (w, r, r), вращение вокруг X (spin), зеркало для левой стороны, поворот
    float cs = cosf(d.spin), sn = sinf(d.spin);
    Mat3 yr = rotYPR(wy, 0, 0);
    V3 cw = mul(bodyRot, w.p) + pos;
    float sx = leftSide ? -w.w : w.w;
    V3 c0 = yr.c0 * sx;
    V3 c1 = (yr.c1 * cs + yr.c2 * sn) * w.r;
    V3 c2 = (yr.c2 * cs - yr.c1 * sn) * w.r;
    const V3 &Rr = R3.R, &Uu = R3.U, &Ff = R3.F;
    V3 v0(dot(c0, Rr), dot(c0, Uu), dot(c0, Ff)), v1(dot(c1, Rr), dot(c1, Uu), dot(c1, Ff)), v2(dot(c2, Rr), dot(c2, Uu), dot(c2, Ff));
    V3 tp = R3.toView(cw);
    if (gTmpV.size() < m.v.size()) gTmpV.resize(m.v.size());
    for (size_t i = 0; i < m.v.size(); i++) { const V3& p = m.v[i]; gTmpV[i] = v0 * p.x + v1 * p.y + v2 * p.z + tp; }
    static const Col wc[3] = {Col(0.07f, 0.07f, 0.08f), Col(0.75f, 0.76f, 0.8f), Col(0.12f, 0.12f, 0.13f)};
    float ff = fogFactor(tp.z);
    for (size_t i = 0; i < m.t.size(); i++) {
        const MeshTri& t = m.t[i];
        V3 nl = t.n; if (leftSide) nl.x = -nl.x;
        V3 nw = norm(yr.c0 * nl.x + (yr.c1 * cs + yr.c2 * sn) * nl.y + (yr.c2 * cs - yr.c1 * sn) * nl.z);
        Col c = shadeN(wc[t.mat], nw, cw);
        c = lerpc(c, L.fog, ff);
        SDL_Color sc = toSDL(c);
        // у левых колёс зеркальная матрица — меняем порядок обхода
        if (leftSide) R3.triView(gTmpV[t.a], gTmpV[t.c], gTmpV[t.b], sc, sc, sc, UV_WHITE, UV_WHITE, UV_WHITE, 2, CAR_CULL, 0);
        else R3.triView(gTmpV[t.a], gTmpV[t.b], gTmpV[t.c], sc, sc, sc, UV_WHITE, UV_WHITE, UV_WHITE, 2, CAR_CULL, 0);
    }
}

// наклейка на кузов (номер, ДПС) в локальных координатах
static void bodyDecal(const Mat3& rot, const V3& pos, V3 c, V3 ax, V3 ay, SDL_FPoint t0, SDL_FPoint t1, SDL_FPoint t2, SDL_FPoint t3, Col col, float bias, float emis = 0) {
    V3 a = mul(rot, c - ax + ay) + pos, b = mul(rot, c + ax + ay) + pos, cc = mul(rot, c + ax - ay) + pos, dd = mul(rot, c - ax - ay) + pos;
    V3 n = norm(cross(b - a, dd - a));
    Col sh = shadeN(col, n, a, emis);
    sh = lerpc(sh, L.fog, fogFactor(R3.toView(a).z));
    SDL_Color sc = toSDL(sh);
    R3.quadWT(a, b, cc, dd, sc, t0, t1, t2, t3, 2, 0, bias);
}

static void drawCarModel(int mi, const DrawCar& d) {
    const CarModel& M = CM[mi];
    if (!R3.sphereVisible(d.pos + V3(0, M.height * 0.5f, 0), M.radius + 0.5f)) return;
    V3 vp = R3.toView(d.pos);
    float dist = sqrtf(vp.x * vp.x + vp.y * vp.y + vp.z * vp.z);
    if (dist > L.fogEnd + M.radius) return;
    int lod = dist < 38 ? 0 : 1;
    Mat3 rot = rotYPR(d.yaw, d.pitch, d.roll);
    Mat3 rotY = rotYPR(d.yaw, 0, 0);
    if (!d.hideBody) {
        drawMesh(M, M.lod[lod], rot, d.pos, d, 0);
        // колёса
        if (dist < 140) for (size_t i = 0; i < M.wheels.size(); i++) drawWheel(M.wheels[i], rotY, d.pos, d.yaw, d, dist < 30 ? 0 : 1, M.wheels[i].p.x < 0);
        // номера
        if (d.plate >= 0 && dist < 60) {
            float pw = 0.26f, ph = 0.06f;
            Col pc(1, 1, 1);
            float pe = d.headOn ? 0.5f : 0;
            if (M.plateF.y > 0) bodyDecal(rot, d.pos, M.plateF, V3(-pw, 0, 0), V3(0, ph, 0), plateUV(d.plate, 0, 0), plateUV(d.plate, 1, 0), plateUV(d.plate, 1, 1), plateUV(d.plate, 0, 1), pc, -0.15f, pe);
            if (M.plateR.y > 0) bodyDecal(rot, d.pos, M.plateR, V3(pw, 0, 0), V3(0, ph, 0), plateUV(d.plate, 0, 0), plateUV(d.plate, 1, 0), plateUV(d.plate, 1, 1), plateUV(d.plate, 0, 1), pc, -0.15f, pe);
        }
        // ливрея ДПС
        if (d.police && M.procStripe && dist < 160 && M.sideZ.size() >= 2) {
            Col blue(0.08f, 0.22f, 0.62f);
            for (int s = -1; s <= 1; s += 2) {
                for (size_t i = 0; i + 1 < M.sideZ.size(); i++) {
                    float x0 = s * (M.sideX[i] + 0.012f), x1 = s * (M.sideX[i + 1] + 0.012f);
                    float hy = 0.075f;
                    V3 a = mul(rot, V3(x0, M.stripeY + hy, M.sideZ[i])) + d.pos, b = mul(rot, V3(x1, M.stripeY + hy, M.sideZ[i + 1])) + d.pos;
                    V3 c2 = mul(rot, V3(x1, M.stripeY - hy, M.sideZ[i + 1])) + d.pos, dd = mul(rot, V3(x0, M.stripeY - hy, M.sideZ[i])) + d.pos;
                    V3 n = mul(rot, V3((float)s, 0, 0));
                    Col sh = lerpc(shadeN(blue, n, a), L.fog, fogFactor(dist));
                    R3.quadW(a, b, c2, dd, toSDL(sh), 2, 0, -0.35f);
                }
            }
        }
        if (d.police && dist < 80) {
            for (int s = -1; s <= 1; s += 2) {
                V3 c(s * (M.doorX + 0.018f), M.dpsY, M.doorZ);
                V3 ax(0, 0, s > 0 ? 0.42f : -0.42f), ay(0, 0.13f, 0);
                bodyDecal(rot, d.pos, c, ax, ay, uvp(DPS_X + 0.5f, DPS_Y + 0.5f), uvp(DPS_X + DPS_W - 0.5f, DPS_Y + 0.5f), uvp(DPS_X + DPS_W - 0.5f, DPS_Y + DPS_H - 0.5f), uvp(DPS_X + 0.5f, DPS_Y + DPS_H - 0.5f), Col(1, 1, 1), -0.45f);
            }
        }
        // мигалка
        if (d.police) {
            float y = M.roofY + 0.02f, z = (M.wheels.empty() ? 0 : -0.15f);
            bool ph = fmodf(d.time, 0.5f) < 0.25f;
            float fr = d.flash ? (ph ? 1.f : 0.25f) : 0.f, fb = d.flash ? (ph ? 0.25f : 1.f) : 0.f;
            Col cr = d.flash ? Col(1, 0.1f, 0.1f) * (0.35f + 0.65f * fr) : Col(0.55f, 0.08f, 0.08f);
            Col cb = d.flash ? Col(0.15f, 0.3f, 1) * (0.35f + 0.65f * fb) : Col(0.08f, 0.12f, 0.5f);
            // основание
            V3 p0 = mul(rot, V3(0, y + 0.03f, z)) + d.pos;
            (void)p0;
            auto lbox = [&](float x0, float x1, Col c, float e) {
                V3 pts[8];
                for (int i = 0; i < 8; i++) pts[i] = mul(rot, V3((i & 1) ? x1 : x0, y + ((i & 2) ? 0.13f : 0), z + ((i & 4) ? 0.13f : -0.13f))) + d.pos;
                int faces[5][4] = {{2, 3, 7, 6}, {4, 5, 7, 6}, {0, 1, 3, 2}, {1, 3, 7, 5}, {0, 2, 6, 4}};
                for (int f = 0; f < 5; f++) {
                    V3 n = norm(cross(pts[faces[f][1]] - pts[faces[f][0]], pts[faces[f][2]] - pts[faces[f][0]]));
                    Col sh = shadeN(c, n, pts[0], e);
                    R3.quadW(pts[faces[f][0]], pts[faces[f][1]], pts[faces[f][2]], pts[faces[f][3]], toSDL(lerpc(sh, L.fog, fogFactor(dist))), 2, 0, -0.05f);
                }
            };
            lbox(-0.55f, -0.08f, cr, d.flash ? 1.f : 0.f);
            lbox(-0.08f, 0.08f, Col(0.85f, 0.85f, 0.9f), 0);
            lbox(0.08f, 0.55f, cb, d.flash ? 1.f : 0.f);
            if (d.flash) {
                float gsz = 0.7f + L.night * 1.6f;
                V3 gl = mul(rot, V3(-0.35f, y + 0.08f, z)) + d.pos, gr = mul(rot, V3(0.35f, y + 0.08f, z)) + d.pos;
                R3.billboard(gl, gsz * fr, gsz * fr, toSDL(Col(1, 0.15f, 0.1f) * (0.5f + 0.5f * L.night)), 0, 3, -0.4f);
                R3.billboard(gr, gsz * fb, gsz * fb, toSDL(Col(0.2f, 0.35f, 1) * (0.5f + 0.5f * L.night)), 0, 3, -0.4f);
            }
        }
        // такси: шашечки на крыше
        if (d.taxi && dist < 120) {
            float y = M.roofY;
            for (int s = -1; s <= 1; s += 2) {
                V3 c(0, y + 0.1f, s * 0.0f); (void)c;
                bodyDecal(rot, d.pos, V3(0, y + 0.1f, s * 0.08f), V3(s > 0 ? -0.3f : 0.3f, 0, 0), V3(0, 0.08f, 0), uvp(CHECK_X + 0.5f, CHECK_Y + 0.5f), uvp(CHECK_X + 15.5f, CHECK_Y + 0.5f), uvp(CHECK_X + 15.5f, CHECK_Y + 7.5f), uvp(CHECK_X + 0.5f, CHECK_Y + 7.5f), Col(1, 1, 1), -0.1f, L.night * 0.6f);
            }
        }
    }
    // свет фар и фонарей ночью
    if (L.night > 0.2f && dist < 260) {
        float k = L.night;
        for (int s = -1; s <= 1; s += 2) {
            if (d.headOn && M.headL.y > 0) {
                V3 hp = mul(rot, V3(M.headL.x * s, M.headL.y, M.headL.z + 0.05f)) + d.pos;
                R3.billboard(hp, 0.45f, 0.45f, toSDL(Col(1, 0.95f, 0.8f) * (0.8f * k)), 0, 3, -0.3f);
            }
            if (M.tailL.y > 0 && (d.headOn || d.brake)) {
                V3 tp = mul(rot, V3(M.tailL.x * s, M.tailL.y, M.tailL.z - 0.05f)) + d.pos;
                float b = d.brake ? 0.9f : 0.45f;
                R3.billboard(tp, 0.35f, 0.35f, toSDL(Col(1, 0.1f, 0.05f) * (b * k)), 0, 3, -0.3f);
            }
        }
        if (d.headOn && dist < 120) {
            // пятно света на дороге
            V3 f = fwdOf(d.yaw), r = rightOf(d.yaw);
            V3 c = d.pos + f * (M.maxZ + 9.0f); c.y = 0.03f;
            R3.groundSprite(c, r * 4.0f, f * 9.5f, toSDL(Col(0.55f, 0.5f, 0.38f) * k), 1, 0);
        }
    }
}

static void loadAllModels() {
    int n = sizeof(CAR_MODELS_SRC) / sizeof(CAR_MODELS_SRC[0]);
    for (int i = 0; i < n; i++) loadEmbeddedModel(CAR_MODELS_SRC[i]);
    buildClassic(); buildMinibus(); buildTruck(); buildTractor(); buildTrailer();
    buildWheelMesh();
}

// =====================================================================
//  Мир: дорожный граф, застройка, светофоры, пешеходы
// =====================================================================
struct Node {
    V3 p; std::vector<int> edges; bool light = false; float trim = 9; float phase = 0; bool city = true;
};
struct Edge {
    int a, b; int lanes; float laneW; float limitKmh; bool highway; float sidewalk;
    V3 dir; float len; float yaw; V3 pa, pb; float slen; float halfW;
    int district;
};
static std::vector<Node> NODES;
static std::vector<Edge> EDGES;

static const char* DISTRICT_NAMES[4] = {"ГОРОД", "ВОСТОЧНАЯ ТРАССА", "СЕВЕРНАЯ ТРАССА", "ЗАПАДНАЯ ТРАССА"};
static const int DISTRICT_RANK[4] = {0, 1, 3, 5};
static int districtAt(const V3& p) {
    if (p.x > -60 && p.x < 390 && p.z > -60 && p.z < 390) return 0;
    if (p.z >= 700) return 2;
    if (p.x >= 390) return 1;
    return 3;
}

static int addNode(float x, float z, bool city) { Node n; n.p = V3(x, 0, z); n.city = city; NODES.push_back(n); return (int)NODES.size() - 1; }
static int addEdge(int a, int b, int lanes, float limit, bool hw) {
    Edge e; e.a = a; e.b = b; e.lanes = lanes; e.laneW = 3.5f; e.limitKmh = limit; e.highway = hw; e.sidewalk = hw ? 0 : 3.5f;
    EDGES.push_back(e); int id = (int)EDGES.size() - 1;
    NODES[a].edges.push_back(id); NODES[b].edges.push_back(id);
    return id;
}
static inline float edgeHalfWidth(const Edge& e) { return e.lanes * e.laneW; }

static void finalizeRoads() {
    for (size_t i = 0; i < NODES.size(); i++) {
        Node& n = NODES[i];
        float mw = 0; for (int ei : n.edges) mw = std::max(mw, edgeHalfWidth(EDGES[ei]) + EDGES[ei].sidewalk);
        if (n.edges.size() == 2 && !n.city) n.trim = 45;
        else if (n.edges.size() == 2) n.trim = mw + 3;
        else n.trim = mw + 1.5f;
        n.light = n.edges.size() >= 3;
        n.phase = (float)(hash32((uint32_t)i * 31 + 7) % 32);
    }
    for (size_t i = 0; i < EDGES.size(); i++) {
        Edge& e = EDGES[i];
        V3 d = NODES[e.b].p - NODES[e.a].p; e.len = len(d); e.dir = d * (1 / e.len);
        e.yaw = atan2f(e.dir.x, e.dir.z);
        e.pa = NODES[e.a].p + e.dir * NODES[e.a].trim;
        e.pb = NODES[e.b].p - e.dir * NODES[e.b].trim;
        e.slen = e.len - NODES[e.a].trim - NODES[e.b].trim;
        e.halfW = edgeHalfWidth(e);
        e.district = districtAt((e.pa + e.pb) * 0.5f);
    }
}

static int CITY_N[4][4];
static void buildRoads() {
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) CITY_N[i][j] = addNode(i * 110.f, j * 110.f, true);
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
        if (i < 3) addEdge(CITY_N[i][j], CITY_N[i + 1][j], 1, 60, false);
        if (j < 3) addEdge(CITY_N[i][j], CITY_N[i][j + 1], 1, 60, false);
    }
    int HA = addNode(800, 110, false), HB = addNode(800, 1400, false), HC = addNode(-700, 1400, false);
    int HF = addNode(-700, 220, false), HD = addNode(-700, -900, false), HE = addNode(800, -900, false);
    addEdge(CITY_N[3][1], HA, 2, 70, true);
    addEdge(HF, CITY_N[0][2], 2, 70, true);
    addEdge(HA, HB, 2, 90, true); addEdge(HB, HC, 2, 90, true); addEdge(HC, HF, 2, 90, true);
    addEdge(HF, HD, 2, 90, true); addEdge(HD, HE, 2, 90, true); addEdge(HE, HA, 2, 90, true);
    finalizeRoads();
}

// --- полосы ---
// dir 0: a->b, 1: b->a. Полоса 0 — ближняя к осевой.
static inline void laneFrame(const Edge& e, int dir, V3& start, V3& fwd, V3& right, float& yaw) {
    if (dir == 0) { start = e.pa; fwd = e.dir; yaw = e.yaw; }
    else { start = e.pb; fwd = -e.dir; yaw = wrapAngle(e.yaw + PI); }
    right = rightOf(yaw);
}
static inline float laneOffset(const Edge& e, int lane) { return (lane + 0.5f) * e.laneW; }
static V3 lanePos(const Edge& e, int dir, int lane, float s, float lat) {
    V3 st, f, r; float y; laneFrame(e, dir, st, f, r, y);
    return st + f * s + r * (laneOffset(e, lane) + lat);
}
static inline int edgeEndNode(const Edge& e, int dir) { return dir == 0 ? e.b : e.a; }

// светофор: ось 0 — дорога вдоль X, 1 — вдоль Z
static float gWorldTime = 0;
static int lightAxisOf(const Edge& e) { return fabsf(e.dir.x) > fabsf(e.dir.z) ? 0 : 1; }
static const float LIGHT_T = 32.0f;
static int lightState(int node, int axis) {   // 0 зелёный, 1 жёлтый, 2 красный
    const Node& n = NODES[node];
    if (!n.light) return 0;
    float t = fmodf(gWorldTime + n.phase, LIGHT_T);
    if (axis == 0) { if (t < 11) return 0; if (t < 13) return 1; return 2; }
    if (t >= 16 && t < 27) return 0;
    if (t >= 27 && t < 29) return 1;
    return 2;
}
static float lightRemaining(int node, int axis) {
    float t = fmodf(gWorldTime + NODES[node].phase, LIGHT_T);
    if (axis == 0) return t < 13 ? 13 - t : LIGHT_T - t;
    return (t >= 16 && t < 29) ? 29 - t : (t < 16 ? 16 - t : LIGHT_T - t + 16);
}

// ближайшая точка дороги к позиции
struct RoadHit { int edge; float s; float lat; float dist; bool onNode; int node; };
static RoadHit nearestRoad(const V3& p) {
    RoadHit best; best.edge = -1; best.dist = 1e9f; best.onNode = false; best.node = -1; best.s = 0; best.lat = 0;
    for (size_t i = 0; i < EDGES.size(); i++) {
        const Edge& e = EDGES[i];
        V3 d = p - e.pa; float s = dot(d, e.dir);
        float sc = clampf(s, 0, e.slen);
        V3 q = e.pa + e.dir * sc;
        float lat = dot(p - q, rightOf(e.yaw));
        float dd = dist2d(p, q);
        if (dd < best.dist) { best.dist = dd; best.edge = (int)i; best.s = sc; best.lat = lat; }
    }
    for (size_t i = 0; i < NODES.size(); i++) {
        float dd = dist2d(p, NODES[i].p);
        if (dd < NODES[i].trim && dd - NODES[i].trim * 0.6f < best.dist) { best.onNode = true; best.node = (int)i; }
    }
    return best;
}
static bool onRoadSurface(const V3& p, float* limitKmh) {
    RoadHit h = nearestRoad(p);
    if (h.onNode) {
        const Node& n = NODES[h.node];
        if (dist2d(p, n.p) < n.trim + 2) {
            if (limitKmh) { *limitKmh = n.city ? 60.f : 90.f; if (!n.city) for (int ei : n.edges) *limitKmh = std::min(*limitKmh, EDGES[ei].limitKmh); }
            if (!n.city && n.edges.size() == 2) {
                // поворот трассы: проверяем дугу
                return true;
            }
            return true;
        }
    }
    if (h.edge >= 0) {
        const Edge& e = EDGES[h.edge];
        if (limitKmh) *limitKmh = e.limitKmh;
        return h.dist < e.halfW + (e.highway ? 2.5f : 0.5f);
    }
    return false;
}

// ---------------------------------------------------------------------
//  Статические объекты
// ---------------------------------------------------------------------
enum ObjType { OB_BUILDING, OB_TREE, OB_LAMP, OB_SIGN, OB_BILLBOARD, OB_POST, OB_BUSH, OB_FENCE, OB_TLIGHT, OB_CAMERA, OB_BENCH, OB_KIOSK, OB_BLOCK };
struct SObj {
    int type; V3 p; V3 h; float yaw; Col col; Col col2; int floors; uint32_t seed; float radius;
    int ival; std::string text; bool collide;
};
static std::vector<SObj> OBJS;
static const float GRID_X0 = -900, GRID_Z0 = -1100, GRID_CS = 60;
static const int GRID_NX = 46, GRID_NZ = 48;
static std::vector<int> GRID[GRID_NX * GRID_NZ];
static inline int gridIdx(float x, float z) {
    int gx = (int)((x - GRID_X0) / GRID_CS), gz = (int)((z - GRID_Z0) / GRID_CS);
    if (gx < 0 || gz < 0 || gx >= GRID_NX || gz >= GRID_NZ) return -1;
    return gz * GRID_NX + gx;
}
static void addObj(const SObj& o) {
    OBJS.push_back(o); int id = (int)OBJS.size() - 1;
    // в каждую ячейку, которую покрывает объект
    float r = o.radius;
    for (float x = o.p.x - r; x <= o.p.x + r + GRID_CS * 0.99f; x += GRID_CS) for (float z = o.p.z - r; z <= o.p.z + r + GRID_CS * 0.99f; z += GRID_CS) {
        int g = gridIdx(std::min(x, o.p.x + r), std::min(z, o.p.z + r));
        if (g >= 0 && (GRID[g].empty() || GRID[g].back() != id)) GRID[g].push_back(id);
    }
}
static bool pointNearRoad(const V3& p, float margin) {
    for (const Edge& e : EDGES) {
        float s = dot(p - e.pa, e.dir); float sc = clampf(s, -NODES[e.a].trim, e.slen + NODES[e.b].trim);
        V3 q = e.pa + e.dir * sc;
        if (dist2d(p, q) < e.halfW + e.sidewalk + margin) return true;
    }
    for (const Node& n : NODES) if (dist2d(p, n.p) < n.trim + margin + (n.city ? 0 : 20)) return true;
    return false;
}

struct Post { int edge, dir; float s; V3 pos; float yaw; int district; };
static Post POSTS[4];
struct Billboard { const char* text; };
static const char* BILLBOARD_TEXTS[] = {"ПРИСТЕГНИСЬ!", "НЕ ГОНИ!", "СОБЛЮДАЙ ДИСТАНЦИЮ", "ПЕШЕХОД ВСЕГДА ПРАВ", "ВЕДЁТСЯ ФОТОФИКСАЦИЯ", "ГИБДД: ТЕЛ. 112", "СКОРОСТЬ УБИВАЕТ"};

static void buildScenery() {
    Rng r(777);
    // городские кварталы
    static const Col facades[] = {hexc(0xd8d2c4), hexc(0xb8b8b4), hexc(0xc9876a), hexc(0xe6dcc0), hexc(0xa9c0d0), hexc(0xd0b090), hexc(0xe8e8e8), hexc(0x9aa59a)};
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        float x0 = i * 110 + 12, x1 = (i + 1) * 110 - 12, z0 = j * 110 + 12, z1 = (j + 1) * 110 - 12;
        bool park = (i == 1 && j == 1);
        if (park) {
            for (int k = 0; k < 26; k++) {
                SObj t; t.type = OB_TREE; t.p = V3(r.range(x0 + 3, x1 - 3), 0, r.range(z0 + 3, z1 - 3)); t.h = V3(0, r.range(5, 9), 0);
                t.col = Col(0.2f + r.f() * 0.1f, 0.45f + r.f() * 0.15f, 0.18f); t.radius = 3; t.seed = r.next(); t.collide = true; t.yaw = 0; t.floors = 0; t.ival = 0;
                addObj(t);
            }
            for (int k = 0; k < 6; k++) {
                SObj b; b.type = OB_BENCH; b.p = V3(x0 + 8 + k * 13, 0, (z0 + z1) * 0.5f + ((k & 1) ? 4 : -4)); b.h = V3(0.9f, 0.45f, 0.25f); b.yaw = 0; b.col = Col(0.5f, 0.32f, 0.18f);
                b.radius = 1; b.collide = true; b.seed = 0; b.floors = 0; b.ival = 0; addObj(b);
            }
            continue;
        }
        // 2-4 здания по периметру квартала
        int layout = r.i(3);
        std::vector<std::pair<V3, V3>> boxes;
        float mx = (x0 + x1) * 0.5f, mz = (z0 + z1) * 0.5f;
        if (layout == 0) {
            boxes.push_back({V3(mx - 21, 0, mz - 21), V3(18, 0, 18)}); boxes.push_back({V3(mx + 21, 0, mz - 21), V3(18, 0, 18)});
            boxes.push_back({V3(mx - 21, 0, mz + 21), V3(18, 0, 18)}); boxes.push_back({V3(mx + 21, 0, mz + 21), V3(18, 0, 18)});
        } else if (layout == 1) {
            boxes.push_back({V3(mx, 0, mz - 24), V3(38, 0, 12)}); boxes.push_back({V3(mx, 0, mz + 24), V3(38, 0, 12)});
        } else {
            boxes.push_back({V3(mx - 24, 0, mz), V3(12, 0, 38)}); boxes.push_back({V3(mx + 22, 0, mz - 18), V3(14, 0, 18)}); boxes.push_back({V3(mx + 22, 0, mz + 24), V3(14, 0, 11)});
        }
        for (auto& bx : boxes) {
            SObj b; b.type = OB_BUILDING; b.p = bx.first; b.yaw = 0;
            b.floors = 3 + r.i(i == 2 && j == 0 ? 14 : 9);
            if (r.chance(0.2f)) b.floors = 2 + r.i(3);
            b.h = V3(bx.second.x, b.floors * 3.2f, bx.second.z);
            b.col = facades[r.i(8)]; b.col2 = Col(0.35f, 0.33f, 0.32f); b.seed = r.next(); b.radius = sqrtf(b.h.x * b.h.x + b.h.z * b.h.z);
            b.collide = true; b.ival = r.chance(0.35f) ? 1 : 0;   // 1 — витрины на первом этаже
            addObj(b);
        }
        // деревья во дворах
        for (int k = 0; k < 4; k++) {
            SObj t; t.type = OB_TREE; t.p = V3(mx + r.range(-6, 6), 0, mz + r.range(-6, 6)); t.h = V3(0, r.range(5, 8), 0);
            t.col = Col(0.22f, 0.5f, 0.2f); t.radius = 3; t.seed = r.next(); t.collide = true; t.yaw = 0; t.floors = 0; t.ival = 0;
            if (layout != 0) addObj(t);
        }
    }
    // фонари и деревья вдоль городских улиц, ларьки
    for (size_t ei = 0; ei < EDGES.size(); ei++) {
        const Edge& e = EDGES[ei];
        V3 rr = rightOf(e.yaw);
        if (!e.highway) {
            for (float s = 10; s < e.slen - 5; s += 25) {
                for (int side = -1; side <= 1; side += 2) {
                    SObj l; l.type = OB_LAMP; l.p = e.pa + e.dir * s + rr * (side * (e.halfW + e.sidewalk - 0.4f)); l.yaw = e.yaw + (side > 0 ? -PI / 2 : PI / 2);
                    l.h = V3(0, 7, 0); l.radius = 1.5f; l.collide = true; l.col = Col(0.3f, 0.3f, 0.32f); l.seed = 0; l.floors = 0; l.ival = side;
                    if (((int)(s / 25) + side) % 2 == 0) addObj(l);
                }
            }
            if (r.chance(0.3f)) {
                SObj k; k.type = OB_KIOSK; float s = e.slen * r.range(0.3f, 0.7f); int side = r.chance(0.5f) ? 1 : -1;
                k.p = e.pa + e.dir * s + rr * (side * (e.halfW + e.sidewalk + 3.0f)); k.yaw = e.yaw + (side > 0 ? -PI / 2 : PI / 2);
                k.h = V3(1.8f, 2.6f, 1.4f); k.col = r.chance(0.5f) ? hexc(0x3070b0) : hexc(0x40a050); k.radius = 2.5f; k.collide = true; k.seed = r.next(); k.floors = 0; k.ival = 0;
                if (!pointNearRoad(k.p, 0.5f)) addObj(k);
            }
        } else {
            // отбойник с обеих сторон, деревья и поля дальше
            for (float s = 0; s < e.slen; s += 9) {
                for (int side = -1; side <= 1; side += 2) {
                    int n = r.i(3);
                    for (int k = 0; k < n; k++) {
                        float off = e.halfW + r.range(14, 80);
                        V3 p = e.pa + e.dir * (s + r.range(0, 9)) + rr * (side * off);
                        if (pointNearRoad(p, 7)) continue;
                        if (districtAt(p) == 0) continue;
                        SObj t; t.type = r.chance(0.25f) ? OB_BUSH : OB_TREE; t.p = p;
                        bool pine = (hash32((uint32_t)(p.x * 0.01f) * 13 + (uint32_t)(p.z * 0.01f + 50) * 7) % 3) == 0;
                        t.h = V3(0, t.type == OB_BUSH ? r.range(1, 2) : r.range(6, 13), 0);
                        t.col = pine ? Col(0.12f, 0.32f, 0.16f) : Col(0.25f + r.f() * 0.12f, 0.45f + r.f() * 0.15f, 0.15f);
                        t.ival = pine ? 1 : 0; t.radius = 3; t.seed = r.next(); t.collide = t.type == OB_TREE; t.yaw = 0; t.floors = 0;
                        addObj(t);
                    }
                }
            }
            for (float s = 120; s < e.slen - 100; s += 450 + r.range(0, 300)) {
                int side = r.chance(0.5f) ? 1 : -1;
                SObj b; b.type = OB_BILLBOARD; b.p = e.pa + e.dir * s + rr * (side * (e.halfW + 12)); b.yaw = e.yaw + (side > 0 ? PI : 0) * 0 + PI * (side > 0 ? 1.f : 0.f);
                b.yaw = (side > 0) ? wrapAngle(e.yaw + PI) : e.yaw;   // щит смотрит навстречу движению по ближней стороне
                b.h = V3(5, 7, 0.3f); b.text = BILLBOARD_TEXTS[r.i(7)]; b.col = r.chance(0.5f) ? hexc(0x1d4fa0) : hexc(0xc03028); b.radius = 6; b.collide = true; b.seed = r.next(); b.floors = 0; b.ival = 0;
                addObj(b);
            }
        }
        // знаки ограничения скорости в начале участка (для обоих направлений)
        for (int dir = 0; dir < 2; dir++) {
            V3 st, f, rgt; float y; laneFrame(e, dir, st, f, rgt, y);
            SObj sg; sg.type = OB_SIGN; sg.p = st + f * 15 + rgt * (e.halfW + (e.highway ? 3.0f : e.sidewalk - 0.6f)); sg.yaw = wrapAngle(y + PI);
            sg.h = V3(0, 2.6f, 0); sg.ival = (int)e.limitKmh; sg.radius = 1; sg.collide = true; sg.col = Col(1, 1, 1); sg.seed = 0; sg.floors = 0;
            addObj(sg);
        }
    }
    // светофоры на узлах
    for (size_t ni = 0; ni < NODES.size(); ni++) {
        const Node& n = NODES[ni];
        if (!n.light) continue;
        for (int ei : n.edges) {
            const Edge& e = EDGES[ei];
            int dir = (e.b == (int)ni) ? 0 : 1;   // направление, подходящее к узлу
            V3 st, f, rgt; float y; laneFrame(e, dir, st, f, rgt, y);
            V3 end = (dir == 0) ? e.pb : e.pa;
            SObj t; t.type = OB_TLIGHT; t.p = end + rgt * (e.halfW + 1.2f) + f * 1.0f; t.yaw = wrapAngle(y + PI);
            t.h = V3(0, 4.2f, 0); t.ival = (int)ni; t.floors = lightAxisOf(e); t.radius = 1; t.collide = true; t.col = Col(0.2f, 0.2f, 0.2f); t.seed = (uint32_t)e.lanes;
            addObj(t);
        }
    }
    // посты ДПС
    struct PD { int a, b; float s; };
    int ne = (int)EDGES.size();
    auto findEdge = [&](int na, int nb) { for (int i = 0; i < ne; i++) if ((EDGES[i].a == na && EDGES[i].b == nb) || (EDGES[i].a == nb && EDGES[i].b == na)) return i; return 0; };
    int city = findEdge(CITY_N[2][1], CITY_N[3][1]);
    int east = 0, north = 0, west = 0;
    for (int i = 0; i < ne; i++) {
        const Edge& e = EDGES[i];
        if (e.highway && fabsf(e.dir.z) > 0.9f && NODES[e.a].p.x > 700 && e.slen > 1000) east = i;
        if (e.highway && fabsf(e.dir.x) > 0.9f && NODES[e.a].p.z > 1300) north = i;
        if (e.highway && fabsf(e.dir.z) > 0.9f && NODES[e.a].p.x < -650 && NODES[e.a].p.z > 200 && NODES[e.b].p.z < 0) west = i;
    }
    int pe[4] = {city, east, north, west}; float ps[4] = {40, 420, 700, 500};
    for (int k = 0; k < 4; k++) {
        Post& P = POSTS[k]; P.edge = pe[k]; P.dir = 0; P.s = std::min(ps[k], EDGES[pe[k]].slen - 30);
        const Edge& e = EDGES[P.edge];
        V3 st, f, rgt; float y; laneFrame(e, P.dir, st, f, rgt, y);
        P.pos = st + f * P.s + rgt * (e.halfW + (e.highway ? 1.9f : 1.6f)); P.yaw = y; P.district = k;
        SObj o; o.type = OB_POST; o.p = st + f * (P.s + 10) + rgt * (e.halfW + (e.highway ? 10.5f : e.sidewalk + 3.5f)); o.yaw = wrapAngle(y - PI / 2);
        o.h = V3(2.2f, 3.0f, 2.0f); o.radius = 4; o.collide = true; o.col = Col(0.95f, 0.95f, 0.95f); o.seed = 0; o.floors = 0; o.ival = k;
        addObj(o);
    }
}

// ---------------------------------------------------------------------
//  Отрисовка мира
// ---------------------------------------------------------------------
static inline SDL_Color fogged(const Col& c, float d) { return toSDL(lerpc(c, L.fog, fogFactor(d))); }
static inline SDL_Color foggedV(const Col& c, const V3& w) { return fogged(c, dist2d(w, R3.pos)); }

static void drawGround() {
    // дальняя плоскость цвета тумана — горизонт
    V3 c = R3.pos; c.y = 0;
    float F = 3000;
    Col far = L.fog;
    R3.quadW(c + V3(-F, -0.05f, -F), c + V3(F, -0.05f, -F), c + V3(F, -0.05f, F), c + V3(-F, -0.05f, F), toSDL(far), 0);
    // клетки травы/полей
    float cs = 40; int n = (int)(std::min(L.fogEnd, 360.f) / cs) + 1;
    int cx = (int)floorf(c.x / cs), cz = (int)floorf(c.z / cs);
    for (int dz = -n; dz <= n; dz++) for (int dx = -n; dx <= n; dx++) {
        float x0 = (cx + dx) * cs, z0 = (cz + dz) * cs;
        V3 cc(x0 + cs * 0.5f, 0, z0 + cs * 0.5f);
        if (!R3.sphereVisible(cc, cs * 0.75f)) continue;
        uint32_t h = hash32((uint32_t)(cx + dx + 1000) * 7919u + (uint32_t)(cz + dz + 1000) * 104729u);
        Col g;
        bool cityArea = (cc.x > -40 && cc.x < 370 && cc.z > -40 && cc.z < 370);
        if (cityArea) g = Col(0.42f, 0.44f, 0.4f);
        else {
            // поля крупными участками
            uint32_t fh = hash32((uint32_t)((int)floorf(cc.x / 160) + 500) * 31u + (uint32_t)((int)floorf(cc.z / 160) + 500) * 977u);
            int kind = fh % 5;
            if (kind == 0) g = Col(0.62f, 0.58f, 0.3f);
            else if (kind == 1) g = Col(0.42f, 0.52f, 0.25f);
            else g = Col(0.30f, 0.46f, 0.22f);
            float v = ((h & 255) / 255.f - 0.5f) * 0.05f;
            g = g + Col(v, v, v * 0.5f);
            if (L.rain > 0) g = g * (1 - 0.15f * L.rain);
        }
        Col lit = mulc(g, L.amb + L.sunCol * (L.sun.y * (1 - L.night * 0.85f)));
        V3 p0(x0, 0, z0), p1(x0 + cs, 0, z0), p2(x0 + cs, 0, z0 + cs), p3(x0, 0, z0 + cs);
        R3.quadWC(p0, p1, p2, p3, foggedV(lit, p0), foggedV(lit, p1), foggedV(lit, p2), foggedV(lit, p3), 0);
    }
}

static Col litFlat(const Col& c) { return mulc(c, L.amb + L.sunCol * (L.sun.y * (1 - L.night * 0.85f))); }

static void flatQuad(const V3& a, const V3& b, const V3& c, const V3& d, const Col& col) {
    Col l = litFlat(col);
    R3.quadWC(a, b, c, d, foggedV(l, a), foggedV(l, b), foggedV(l, c), foggedV(l, d), 0);
}

static void drawRoads() {
    Col asph = L.rain > 0.3f ? Col(0.2f, 0.2f, 0.22f) : Col(0.27f, 0.27f, 0.29f);
    Col walk = Col(0.6f, 0.6f, 0.58f), curb = Col(0.75f, 0.75f, 0.72f), white = Col(0.92f, 0.92f, 0.9f), shoulder = Col(0.38f, 0.36f, 0.33f);
    float maxD = L.fogEnd + 40;
    V3 cam = R3.pos;
    // 1) покрытие участков
    for (size_t ei = 0; ei < EDGES.size(); ei++) {
        const Edge& e = EDGES[ei];
        V3 rr = rightOf(e.yaw);
        float seg = 30;
        int ns = (int)ceilf(e.slen / seg);
        for (int k = 0; k < ns; k++) {
            float s0 = k * seg, s1 = std::min(e.slen, (k + 1) * seg);
            V3 c = e.pa + e.dir * ((s0 + s1) * 0.5f);
            if (dist2d(c, cam) > maxD + seg) continue;
            if (!R3.sphereVisible(c, seg * 0.6f + e.halfW + 6)) continue;
            V3 a = e.pa + e.dir * s0, b = e.pa + e.dir * s1;
            float w = e.halfW;
            V3 y1(0, 0.01f, 0);
            if (e.highway) {
                flatQuad(a - rr * (w + 3) + y1, b - rr * (w + 3) + y1, b + rr * (w + 3) + y1, a + rr * (w + 3) + y1, shoulder);
                flatQuad(a - rr * w + y1 * 2, b - rr * w + y1 * 2, b + rr * w + y1 * 2, a + rr * w + y1 * 2, asph);
            } else {
                flatQuad(a - rr * (w + e.sidewalk) + y1, b - rr * (w + e.sidewalk) + y1, b + rr * (w + e.sidewalk) + y1, a + rr * (w + e.sidewalk) + y1, walk);
                flatQuad(a - rr * (w + 0.2f) + y1 * 2, b - rr * (w + 0.2f) + y1 * 2, b + rr * (w + 0.2f) + y1 * 2, a + rr * (w + 0.2f) + y1 * 2, curb);
                flatQuad(a - rr * w + y1 * 3, b - rr * w + y1 * 3, b + rr * w + y1 * 3, a + rr * w + y1 * 3, asph);
            }
        }
    }
    // 2) узлы
    for (size_t ni = 0; ni < NODES.size(); ni++) {
        const Node& n = NODES[ni];
        if (dist2d(n.p, cam) > maxD + n.trim) continue;
        if (!R3.sphereVisible(n.p, n.trim * 1.5f + 8)) continue;
        if (n.city) {
            float t = n.trim, w = t + 0.01f;
            flatQuad(n.p + V3(-w, 0.01f, -w), n.p + V3(w, 0.01f, -w), n.p + V3(w, 0.01f, w), n.p + V3(-w, 0.01f, w), walk);
            float mw = 0; for (int ei : n.edges) mw = std::max(mw, EDGES[ei].halfW);
            // асфальт перекрёстка: крест
            for (int ei : n.edges) {
                const Edge& e = EDGES[ei];
                V3 d = (e.a == (int)ni) ? e.dir : -e.dir; V3 rr = rightOf(atan2f(d.x, d.z));
                flatQuad(n.p - rr * e.halfW - d * mw + V3(0, 0.03f, 0), n.p + d * (t + 0.05f) - rr * e.halfW + V3(0, 0.03f, 0), n.p + d * (t + 0.05f) + rr * e.halfW + V3(0, 0.03f, 0), n.p + rr * e.halfW - d * mw + V3(0, 0.03f, 0), asph);
            }
            flatQuad(n.p + V3(-mw, 0.03f, -mw), n.p + V3(mw, 0.03f, -mw), n.p + V3(mw, 0.03f, mw), n.p + V3(-mw, 0.03f, mw), asph);
        } else {
            // поворот/развязка трассы: веер из полос
            if (n.edges.size() == 2) {
                const Edge& e0 = EDGES[n.edges[0]]; const Edge& e1 = EDGES[n.edges[1]];
                V3 d0 = (e0.a == (int)ni) ? e0.dir : -e0.dir, d1 = (e1.a == (int)ni) ? e1.dir : -e1.dir;
                V3 p0 = n.p + d0 * n.trim, p1 = n.p + d1 * n.trim;
                float w = e0.halfW;
                int N = 10;
                V3 prevL, prevR;
                for (int k = 0; k <= N; k++) {
                    float t = k / (float)N;
                    V3 q = p0 * ((1 - t) * (1 - t)) + n.p * (2 * t * (1 - t)) + p1 * (t * t);
                    V3 tg = norm((n.p - p0) * (2 * (1 - t)) + (p1 - n.p) * (2 * t));
                    V3 rr(tg.z, 0, -tg.x);
                    V3 cl = q - rr * (w + 3), cr = q + rr * (w + 3);
                    V3 al = q - rr * w, ar = q + rr * w;
                    if (k > 0) {
                        flatQuad(prevL + V3(0, 0.01f, 0), cl + V3(0, 0.01f, 0), cr + V3(0, 0.01f, 0), prevR + V3(0, 0.01f, 0), shoulder);
                    }
                    prevL = cl; prevR = cr;
                    (void)al; (void)ar;
                }
                for (int k = 0; k < N; k++) {
                    float t0 = k / (float)N, t1 = (k + 1) / (float)N;
                    V3 q0 = p0 * ((1 - t0) * (1 - t0)) + n.p * (2 * t0 * (1 - t0)) + p1 * (t0 * t0);
                    V3 q1 = p0 * ((1 - t1) * (1 - t1)) + n.p * (2 * t1 * (1 - t1)) + p1 * (t1 * t1);
                    V3 g0 = norm((n.p - p0) * (2 * (1 - t0)) + (p1 - n.p) * (2 * t0)), g1 = norm((n.p - p0) * (2 * (1 - t1)) + (p1 - n.p) * (2 * t1));
                    V3 r0(g0.z, 0, -g0.x), r1(g1.z, 0, -g1.x);
                    flatQuad(q0 - r0 * w + V3(0, 0.02f, 0), q1 - r1 * w + V3(0, 0.02f, 0), q1 + r1 * w + V3(0, 0.02f, 0), q0 + r0 * w + V3(0, 0.02f, 0), asph);
                }
            } else {
                float t = n.trim + 0.5f;
                flatQuad(n.p + V3(-t, 0.02f, -t), n.p + V3(t, 0.02f, -t), n.p + V3(t, 0.02f, t), n.p + V3(-t, 0.02f, t), asph);
            }
        }
    }
    // 3) разметка
    for (size_t ei = 0; ei < EDGES.size(); ei++) {
        const Edge& e = EDGES[ei];
        V3 rr = rightOf(e.yaw);
        V3 y(0, 0.05f, 0);
        float near0 = dot(cam - e.pa, e.dir);
        float sFrom = clampf(near0 - maxD, 0, e.slen), sTo = clampf(near0 + maxD, 0, e.slen);
        if (dist2d(cam, e.pa + e.dir * clampf(near0, 0, e.slen)) > maxD) continue;
        // осевая
        if (e.highway) {
            for (float s = sFrom; s < sTo; s += 30) {
                float s1 = std::min(sTo, s + 30);
                V3 a = e.pa + e.dir * s, b = e.pa + e.dir * s1;
                if (!R3.sphereVisible((a + b) * 0.5f, 20)) continue;
                for (int k = -1; k <= 1; k += 2) {
                    V3 o = rr * (k * 0.15f);
                    flatQuad(a + o - rr * 0.06f + y, b + o - rr * 0.06f + y, b + o + rr * 0.06f + y, a + o + rr * 0.06f + y, white);
                    // края проезжей части
                    V3 o2 = rr * (k * (e.halfW - 0.3f));
                    flatQuad(a + o2 - rr * 0.1f + y, b + o2 - rr * 0.1f + y, b + o2 + rr * 0.1f + y, a + o2 + rr * 0.1f + y, white);
                }
            }
            // прерывистая между полосами
            float st = floorf(sFrom / 12) * 12;
            for (float s = st; s < sTo; s += 12) {
                V3 a = e.pa + e.dir * s, b = e.pa + e.dir * std::min(s + 4, e.slen);
                if (dist2d(a, cam) > 140) continue;
                for (int k = -1; k <= 1; k += 2) {
                    V3 o = rr * (k * e.laneW);
                    flatQuad(a + o - rr * 0.08f + y, b + o - rr * 0.08f + y, b + o + rr * 0.08f + y, a + o + rr * 0.08f + y, white);
                }
            }
        } else {
            float st = floorf(sFrom / 6) * 6;
            for (float s = st; s < sTo - 6; s += 6) {
                if (s < 8 || s > e.slen - 10) continue;
                V3 a = e.pa + e.dir * s, b = e.pa + e.dir * (s + 3);
                if (dist2d(a, cam) > 120) continue;
                flatQuad(a - rr * 0.07f + y, b - rr * 0.07f + y, b + rr * 0.07f + y, a + rr * 0.07f + y, white);
            }
            // сплошная перед перекрёстком, стоп-линии и «зебры»
            for (int end = 0; end < 2; end++) {
                int node = end == 0 ? e.a : e.b;
                V3 base = end == 0 ? e.pa : e.pb; V3 inw = end == 0 ? e.dir : -e.dir;
                if (dist2d(base, cam) > 150) continue;
                flatQuad(base + inw * 8 - rr * 0.07f + y, base + inw * 4.6f - rr * 0.07f + y, base + inw * 4.6f + rr * 0.07f + y, base + inw * 8 - rr * 0.07f + y, white);
                if (NODES[node].light) {
                    // стоп-линия — на полосе, подходящей к узлу (справа по ходу к узлу)
                    V3 rIn = -rr * (end == 0 ? -1.f : 1.f);
                    (void)rIn;
                    V3 approachR = (end == 0) ? -rr : rr;   // правая сторона для движения к узлу
                    flatQuad(base + inw * 4.6f + y, base + inw * 4.2f + y, base + inw * 4.2f + approachR * e.halfW + y, base + inw * 4.6f + approachR * e.halfW + y, white);
                    for (float x = -e.halfW + 0.3f; x < e.halfW - 0.3f; x += 1.0f) {
                        V3 p = base + inw * 0.6f + rr * x;
                        flatQuad(p + y, p + inw * 3.2f + y, p + inw * 3.2f + rr * 0.5f + y, p + rr * 0.5f + y, white);
                    }
                }
            }
        }
    }
}

// --- объекты ---
static void shadedBox(const V3& c, const V3& h, float yaw, const Col& col, int cull = -1, const Col* top = nullptr, float bias = 0) {
    V3 f = fwdOf(yaw), r = rightOf(yaw), u(0, 1, 0);
    V3 p[8];
    for (int i = 0; i < 8; i++) p[i] = c + r * ((i & 1) ? h.x : -h.x) + u * ((i & 2) ? h.y : -h.y) + f * ((i & 4) ? h.z : -h.z);
    struct F4 { int a, b, c, d; V3 n; };
    F4 fs[5] = {{1, 5, 7, 3, r}, {0, 2, 6, 4, -r}, {2, 3, 7, 6, u}, {4, 6, 7, 5, f}, {0, 1, 3, 2, -f}};
    float d = dist2d(c, R3.pos);
    for (int k = 0; k < 5; k++) {
        Col base = (k == 2 && top) ? *top : col;
        Col s = shadeN(base, fs[k].n, c);
        SDL_Color sc = fogged(s, d);
        R3.quadW(p[fs[k].a], p[fs[k].b], p[fs[k].c], p[fs[k].d], sc, 2, 0, bias);
        (void)cull;
    }
}

static void drawText3D(const V3& origin, const V3& right, const V3& up, const char* str, float px, SDL_Color c, float bias = -0.05f) {
    // origin — центр строки
    std::vector<uint32_t> cps = utf8Decode(str);
    float w = (cps.size() * 6 - 1) * px;
    V3 o = origin - right * (w * 0.5f) + up * (3.5f * px);
    for (size_t i = 0; i < cps.size(); i++) {
        if (cps[i] != ' ') {
            int g = glyphIndex(cps[i]); int gx, gy; glyphCell(g, gx, gy);
            V3 a = o + right * (i * 6 * px), b = a + right * (5 * px), cc = b - up * (7 * px), d = a - up * (7 * px);
            R3.quadWT(a, b, cc, d, c, uvp(gx, gy), uvp(gx + 5, gy), uvp(gx + 5, gy + 7), uvp(gx, gy + 7), 2, 0, bias);
        }
    }
}

static void drawBuilding(const SObj& b, float dist) {
    V3 c = b.p;
    float hx = b.h.x, hz = b.h.z, H = b.h.y;
    V3 corners[4] = {c + V3(-hx, 0, -hz), c + V3(hx, 0, -hz), c + V3(hx, 0, hz), c + V3(-hx, 0, hz)};
    V3 normals[4] = {V3(0, 0, -1), V3(1, 0, 0), V3(0, 0, 1), V3(-1, 0, 0)};
    for (int w = 0; w < 4; w++) {
        V3 a = corners[w], bb = corners[(w + 1) % 4];
        V3 n = normals[w];
        if (dot(R3.pos - a, n) <= 0) continue;   // стена смотрит от камеры
        float wl = dist2d(a, bb);
        Col s = shadeN(b.col, n, a + V3(0, H * 0.5f, 0));
        if (dist > 170) {
            SDL_Color sc = fogged(s * 0.9f, dist);
            R3.quadW(a, bb, bb + V3(0, H, 0), a + V3(0, H, 0), sc, 2, 0, 0);
            continue;
        }
        int cols = std::max(1, (int)(wl / 5.0f));
        float cw = wl / cols;
        V3 dirw = (bb - a) * (1.0f / wl);
        for (int fl = 0; fl < b.floors; fl++) {
            float y0 = fl * 3.2f, y1 = y0 + 3.2f;
            for (int k = 0; k < cols; k++) {
                V3 p0 = a + dirw * (k * cw), p1 = a + dirw * ((k + 1) * cw);
                uint32_t hsh = hash32(b.seed + fl * 131 + k * 7 + w * 1013);
                int tile = 0;
                if (fl == 0 && b.ival == 1) tile = 3;
                else if (L.night > 0.5f) tile = (hsh % 100) < 40 ? 1 : 2;
                float emis = (tile == 1 || (tile == 3 && L.night > 0.5f)) ? L.night * 0.9f : 0;
                Col cc = lerpc(s, b.col, emis);
                if (tile == 1 || tile == 3) cc = lerpc(s, Col(1, 1, 1), emis);
                float pd = dist2d(p0, R3.pos);
                SDL_Color sc = fogged(cc, pd);
                float tx = WIN_X + tile * 16;
                R3.quadWT(p0 + V3(0, y1, 0), p1 + V3(0, y1, 0), p1 + V3(0, y0, 0), p0 + V3(0, y0, 0), sc, uvp(tx + 0.5f, WIN_Y + 0.5f), uvp(tx + 15.5f, WIN_Y + 0.5f), uvp(tx + 15.5f, WIN_Y + 15.5f), uvp(tx + 0.5f, WIN_Y + 15.5f), 2, 0, 0);
            }
        }
    }
    // крыша и парапет
    Col roof = shadeN(b.col2, V3(0, 1, 0), c);
    if (R3.pos.y > H - 1 || true) {
        SDL_Color sc = fogged(roof, dist);
        R3.quadW(corners[0] + V3(0, H, 0), corners[1] + V3(0, H, 0), corners[2] + V3(0, H, 0), corners[3] + V3(0, H, 0), sc, 2, 0, 0.5f);
    }
}

static void drawTree(const SObj& t, float dist) {
    float h = t.h.y;
    Col trunk(0.35f, 0.25f, 0.17f);
    float tw = 0.18f + h * 0.01f;
    shadedBox(t.p + V3(0, h * 0.18f, 0), V3(tw, h * 0.18f, tw), 0, trunk);
    int sides = dist < 80 ? 7 : 5;
    float rot = (t.seed % 100) * 0.06f;
    if (t.ival == 1) {
        // ель — три яруса конусов
        for (int tier = 0; tier < 3; tier++) {
            float y0 = h * (0.2f + tier * 0.22f), y1 = y0 + h * 0.42f, rr = h * (0.3f - tier * 0.07f);
            V3 top = t.p + V3(0, y1, 0);
            for (int i = 0; i < sides; i++) {
                float a0 = rot + i * 2 * PI / sides, a1 = rot + (i + 1) * 2 * PI / sides;
                V3 p0 = t.p + V3(cosf(a0) * rr, y0, sinf(a0) * rr), p1 = t.p + V3(cosf(a1) * rr, y0, sinf(a1) * rr);
                V3 n = norm(cross(p1 - p0, top - p0)); if (n.y < 0) n = -n;
                Col s = shadeN(t.col * (0.85f + 0.15f * (i & 1)), n, top);
                R3.triW(p0, p1, top, fogged(s, dist), 2, 0, 0);
            }
        }
    } else {
        // лиственное — «шар» из двух пирамид
        float y0 = h * 0.35f, ym = h * 0.65f, y1 = h * 1.0f, rr = h * 0.33f;
        V3 top = t.p + V3(0, y1, 0), bot = t.p + V3(0, y0, 0);
        for (int i = 0; i < sides; i++) {
            float a0 = rot + i * 2 * PI / sides, a1 = rot + (i + 1) * 2 * PI / sides;
            V3 p0 = t.p + V3(cosf(a0) * rr, ym, sinf(a0) * rr), p1 = t.p + V3(cosf(a1) * rr, ym, sinf(a1) * rr);
            V3 n1 = norm(cross(p1 - p0, top - p0)); if (n1.y < 0) n1 = -n1;
            V3 n2 = norm(cross(p0 - p1, bot - p1)); if (n2.y > 0) n2 = -n2;
            Col c1 = t.col * (0.9f + 0.1f * (i & 1));
            R3.triW(p0, p1, top, fogged(shadeN(c1, n1, top), dist), 2, 0, 0);
            R3.triW(p0, p1, bot, fogged(shadeN(c1 * 0.8f, n2, bot), dist), 2, 0, 0);
        }
    }
}

static void drawTrafficLight(const SObj& o, float dist) {
    V3 f = fwdOf(o.yaw), r = rightOf(o.yaw);
    Col pole(0.25f, 0.25f, 0.27f);
    shadedBox(o.p + V3(0, 2.2f, 0), V3(0.08f, 2.2f, 0.08f), o.yaw, pole);
    // консоль над дорогой
    float arm = o.seed >= 2 ? 6.0f : 3.0f;
    V3 armC = o.p + r * (-arm * 0.5f) + V3(0, 4.6f, 0);
    shadedBox(armC, V3(arm * 0.5f, 0.06f, 0.06f), o.yaw, pole);
    V3 boxC = o.p + r * (-arm + 0.3f) + V3(0, 4.1f, 0);
    shadedBox(boxC, V3(0.2f, 0.55f, 0.15f), o.yaw, Col(0.12f, 0.12f, 0.12f));
    V3 boxC2 = o.p + V3(0, 2.6f, 0) + f * 0.12f;
    shadedBox(boxC2, V3(0.17f, 0.45f, 0.12f), o.yaw, Col(0.12f, 0.12f, 0.12f));
    int st = lightState(o.ival, o.floors);
    Col lc[3] = {Col(1, 0.15f, 0.1f), Col(1, 0.8f, 0.1f), Col(0.2f, 1, 0.4f)};
    int on = st == 0 ? 2 : (st == 1 ? 1 : 0);
    for (int k = 0; k < 3; k++) {
        bool lit = (k == on);
        Col c = lit ? lc[k] : lc[k] * 0.18f;
        for (int b2 = 0; b2 < 2; b2++) {
            V3 bc = b2 == 0 ? boxC : boxC2;
            float step = b2 == 0 ? 0.34f : 0.28f;
            V3 lp = bc + f * (b2 == 0 ? 0.16f : 0.13f) + V3(0, step * (1 - k), 0);
            float s = b2 == 0 ? 0.12f : 0.1f;
            R3.quadW(lp - r * s + V3(0, s, 0), lp + r * s + V3(0, s, 0), lp + r * s - V3(0, s, 0), lp - r * s - V3(0, s, 0), fogged(c, dist * (lit ? 0.4f : 1)), 2, 0, -0.1f);
            if (lit && dist < 220) R3.billboard(lp + f * 0.05f, 0.45f + L.night * 0.5f, 0.45f + L.night * 0.5f, toSDL(c * (0.35f + 0.5f * L.night)), 0, 3, -0.3f);
        }
    }
}

static void drawSign(const SObj& o, float dist) {
    V3 f = fwdOf(o.yaw), r = rightOf(o.yaw);
    shadedBox(o.p + V3(0, 1.2f, 0), V3(0.04f, 1.2f, 0.04f), o.yaw, Col(0.5f, 0.5f, 0.52f));
    V3 c = o.p + V3(0, 2.55f, 0) + f * 0.06f;
    // красный круг с белым центром
    int N = 14; float R1 = 0.38f, R2 = 0.29f;
    Col red = shadeN(Col(0.85f, 0.1f, 0.1f), f, c), wh = shadeN(Col(0.95f, 0.95f, 0.95f), f, c);
    for (int i = 0; i < N; i++) {
        float a0 = i * 2 * PI / N, a1 = (i + 1) * 2 * PI / N;
        V3 p0 = c + r * (cosf(a0) * R1) + V3(0, sinf(a0) * R1, 0), p1 = c + r * (cosf(a1) * R1) + V3(0, sinf(a1) * R1, 0);
        V3 q0 = c + r * (cosf(a0) * R2) + V3(0, sinf(a0) * R2, 0) + f * 0.01f, q1 = c + r * (cosf(a1) * R2) + V3(0, sinf(a1) * R2, 0) + f * 0.01f;
        R3.triW(c, p0, p1, fogged(red, dist), 2, 0, 0);
        R3.triW(c + f * 0.01f, q0, q1, fogged(wh, dist), 2, 0, -0.02f);
    }
    if (dist < 90 && dot(R3.pos - c, f) > 0) { std::string s = fmt("%d", o.ival); drawText3D(c + f * 0.03f, -r, V3(0, 1, 0), s.c_str(), 0.045f, fogged(Col(0.05f, 0.05f, 0.05f), dist), -0.06f); }
}

static void drawLamp(const SObj& o, float dist) {
    V3 f = fwdOf(o.yaw);
    Col pole(0.32f, 0.33f, 0.35f);
    shadedBox(o.p + V3(0, 3.5f, 0), V3(0.07f, 3.5f, 0.07f), o.yaw, pole);
    shadedBox(o.p + V3(0, 6.95f, 0) + f * 0.9f, V3(0.06f, 0.05f, 0.9f), o.yaw, pole);
    V3 lp = o.p + V3(0, 6.85f, 0) + f * 1.7f;
    Col lc = L.night > 0.3f ? Col(1, 0.85f, 0.55f) : Col(0.7f, 0.7f, 0.7f);
    shadedBox(lp, V3(0.18f, 0.06f, 0.3f), o.yaw, lc, -1, nullptr, 0);
    if (L.night > 0.3f) {
        R3.billboard(lp - V3(0, 0.15f, 0), 1.0f, 1.0f, toSDL(Col(1, 0.75f, 0.4f) * (0.5f * L.night)), 0, 3, -0.2f);
        if (dist < 160) R3.groundSprite(V3(lp.x, 0.06f, lp.z), V3(5.5f, 0, 0), V3(0, 0, 5.5f), toSDL(Col(0.45f, 0.36f, 0.2f) * L.night), 1, 0);
    }
}

static void drawBillboard(const SObj& o, float dist) {
    V3 f = fwdOf(o.yaw), r = rightOf(o.yaw);
    Col pole(0.4f, 0.4f, 0.42f);
    shadedBox(o.p + V3(-2.5f, 0, 0) * 0 + r * -2.5f + V3(0, 2.5f, 0), V3(0.12f, 2.5f, 0.12f), o.yaw, pole);
    shadedBox(o.p + r * 2.5f + V3(0, 2.5f, 0), V3(0.12f, 2.5f, 0.12f), o.yaw, pole);
    V3 c = o.p + V3(0, 6.2f, 0);
    shadedBox(c, V3(5.2f, 1.6f, 0.12f), o.yaw, Col(0.85f, 0.85f, 0.85f));
    Col bg = shadeN(o.col, f, c, L.night * 0.5f);
    R3.quadW(c + f * 0.13f - r * 5 + V3(0, 1.45f, 0), c + f * 0.13f + r * 5 + V3(0, 1.45f, 0), c + f * 0.13f + r * 5 - V3(0, 1.45f, 0), c + f * 0.13f - r * 5 - V3(0, 1.45f, 0), fogged(bg, dist), 2, 0, -0.02f);
    if (dist < 220 && dot(R3.pos - c, f) > 0) {
        float px = std::min(0.17f, 9.6f / (utf8Len(o.text.c_str()) * 6.0f));
        drawText3D(c + f * 0.15f, -r, V3(0, 1, 0), o.text.c_str(), px, fogged(Col(1, 1, 1) * (0.8f + 0.2f * L.night), dist), -0.08f);
    }
}

static void drawPostBooth(const SObj& o, float dist) {
    V3 f = fwdOf(o.yaw), r = rightOf(o.yaw);
    Col wall(0.94f, 0.94f, 0.96f), stripe(0.1f, 0.25f, 0.65f);
    shadedBox(o.p + V3(0, 1.5f, 0), V3(2.2f, 1.5f, 2.0f), o.yaw, wall, -1, nullptr);
    for (int k = 0; k < 4; k++) {
        V3 n = k == 0 ? f : (k == 1 ? -f : (k == 2 ? r : -r));
        V3 t = k < 2 ? r : f; float hw = k < 2 ? 2.2f : 2.0f; float off = k < 2 ? 2.01f : 2.21f;
        V3 c = o.p + n * off + V3(0, 1.6f, 0);
        R3.quadW(c - t * hw + V3(0, 0.22f, 0), c + t * hw + V3(0, 0.22f, 0), c + t * hw - V3(0, 0.22f, 0), c - t * hw - V3(0, 0.22f, 0), fogged(shadeN(stripe, n, c), dist), 2, 0, -0.08f);
    }
    shadedBox(o.p + V3(0, 3.15f, 0), V3(2.5f, 0.15f, 2.3f), o.yaw, Col(0.3f, 0.3f, 0.35f));
    // окна
    Col gl = shadeN(Col(0.2f, 0.3f, 0.4f), f, o.p, L.night * 0.6f);
    if (L.night > 0.4f) gl = Col(1, 0.9f, 0.6f) * 0.8f;
    R3.quadW(o.p + f * 2.03f - r * 1.6f + V3(0, 2.6f, 0), o.p + f * 2.03f + r * 1.6f + V3(0, 2.6f, 0), o.p + f * 2.03f + r * 1.6f + V3(0, 2.0f, 0), o.p + f * 2.03f - r * 1.6f + V3(0, 2.0f, 0), fogged(gl, dist), 2, 0, -0.05f);
    if (dist < 160 && dot(R3.pos - o.p, f) > 0) drawText3D(o.p + f * 2.05f + V3(0, 0.75f, 0), -r, V3(0, 1, 0), "ДПС", 0.09f, fogged(Col(0.1f, 0.2f, 0.6f), dist), -0.1f);
    // антенна и флажок
    shadedBox(o.p + V3(1.8f, 4.2f, 1.6f), V3(0.03f, 1.0f, 0.03f), o.yaw, Col(0.3f, 0.3f, 0.3f));
}

static void drawKiosk(const SObj& o, float dist) {
    V3 f = fwdOf(o.yaw), r = rightOf(o.yaw);
    shadedBox(o.p + V3(0, 1.3f, 0), V3(1.8f, 1.3f, 1.4f), o.yaw, Col(0.9f, 0.9f, 0.88f));
    shadedBox(o.p + V3(0, 2.75f, 0), V3(1.95f, 0.15f, 1.55f), o.yaw, o.col);
    Col gl = L.night > 0.4f ? Col(1, 0.92f, 0.7f) : shadeN(Col(0.3f, 0.4f, 0.5f), f, o.p);
    R3.quadW(o.p + f * 1.42f - r * 1.5f + V3(0, 2.2f, 0), o.p + f * 1.42f + r * 1.5f + V3(0, 2.2f, 0), o.p + f * 1.42f + r * 1.5f + V3(0, 1.0f, 0), o.p + f * 1.42f - r * 1.5f + V3(0, 1.0f, 0), fogged(gl, dist), 2, 0, -0.05f);
    if (dist < 100 && dot(R3.pos - o.p, f) > 0) drawText3D(o.p + f * 1.56f + V3(0, 2.75f, 0), -r, V3(0, 1, 0), (o.seed & 1) ? "ПРОДУКТЫ" : "ШАУРМА", 0.05f, fogged(Col(1, 1, 1), dist), -0.1f);
}

static void drawBench(const SObj& o, float dist) {
    (void)dist;
    shadedBox(o.p + V3(0, 0.45f, 0), V3(0.9f, 0.04f, 0.22f), o.yaw, o.col);
    shadedBox(o.p + V3(0, 0.75f, -0.2f), V3(0.9f, 0.18f, 0.03f), o.yaw, o.col);
    shadedBox(o.p + V3(-0.8f, 0.22f, 0), V3(0.04f, 0.22f, 0.2f), o.yaw, Col(0.2f, 0.2f, 0.2f));
    shadedBox(o.p + V3(0.8f, 0.22f, 0), V3(0.04f, 0.22f, 0.2f), o.yaw, Col(0.2f, 0.2f, 0.2f));
}

static void drawGuardRails() {
    V3 cam = R3.pos;
    Col rail(0.72f, 0.74f, 0.76f), post(0.45f, 0.45f, 0.47f);
    for (size_t ei = 0; ei < EDGES.size(); ei++) {
        const Edge& e = EDGES[ei];
        if (!e.highway) continue;
        float near0 = dot(cam - e.pa, e.dir);
        if (dist2d(cam, e.pa + e.dir * clampf(near0, 0, e.slen)) > 160) continue;
        V3 rr = rightOf(e.yaw);
        float s0 = clampf(near0 - 160, 0, e.slen), s1 = clampf(near0 + 160, 0, e.slen);
        for (float s = floorf(s0 / 16) * 16; s < s1; s += 16) {
            float sb = std::min(s + 16, s1);
            for (int side = -1; side <= 1; side += 2) {
                V3 a = e.pa + e.dir * s + rr * (side * (e.halfW + 3.3f)), b = e.pa + e.dir * sb + rr * (side * (e.halfW + 3.3f));
                if (!R3.sphereVisible((a + b) * 0.5f, 10)) continue;
                bool nearPost = false; for (int k = 0; k < 4; k++) if (dist2d((a + b) * 0.5f, POSTS[k].pos) < 45) nearPost = true;
                if (nearPost) continue;
                float d = dist2d(a, cam);
                V3 n = rr * (float)-side;
                Col s1c = shadeN(rail, n, a);
                R3.quadW(a + V3(0, 0.85f, 0), b + V3(0, 0.85f, 0), b + V3(0, 0.55f, 0), a + V3(0, 0.55f, 0), fogged(s1c, d), 2, 0, 0);
                if (d < 90) for (float k = 0; k < 16; k += 4) {
                    V3 p = a + e.dir * k;
                    R3.quadW(p + V3(0, 0.55f, 0) - n * 0.05f, p + V3(0, 0.55f, 0) + e.dir * 0.12f - n * 0.05f, p + e.dir * 0.12f - n * 0.05f, p - n * 0.05f, fogged(shadeN(post, n, p), d), 2, 0, 0.05f);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------
//  Пешеходы
// ---------------------------------------------------------------------
struct Ped {
    bool alive = false;
    V3 pos, target; float yaw = 0, speed = 1.3f, anim = 0;
    int edge = -1, side = 1; int toNode = -1; int state = 0;   // 0 идёт по тротуару, 1 ждёт у перехода, 2 переходит
    int crossEdge = -1, crossNode = -1;
    Col shirt, pants, skin, hair; float h = 1.75f; float wait = 0;
};
static std::vector<Ped> PEDS;

static V3 sidewalkPoint(const Edge& e, int side, float s) {
    V3 rr = rightOf(e.yaw);
    return e.pa + e.dir * s + rr * (side * (e.halfW + e.sidewalk * 0.55f));
}
static void pedPickTarget(Ped& p) {
    const Edge& e = EDGES[p.edge];
    p.target = sidewalkPoint(e, p.side, p.toNode == e.b ? e.slen + 2.0f : -2.0f);
}
static void spawnPed(Ped& p, const V3& near) {
    // случайный городской участок рядом
    for (int tries = 0; tries < 20; tries++) {
        int ei = grng.i((int)EDGES.size());
        const Edge& e = EDGES[ei];
        if (e.highway) continue;
        float s = grng.range(5, e.slen - 5);
        int side = grng.chance(0.5f) ? 1 : -1;
        V3 pos = sidewalkPoint(e, side, s);
        float d = dist2d(pos, near);
        if (d < 40 || d > 160) continue;
        p.alive = true; p.edge = ei; p.side = side; p.pos = pos; p.toNode = grng.chance(0.5f) ? e.a : e.b; p.state = 0;
        static const Col shirts[] = {hexc(0x2a4f8a), hexc(0x8a2a2a), hexc(0x2f6b3a), hexc(0x444444), hexc(0xd8c070), hexc(0xe0e0e0), hexc(0x6a3a7a)};
        p.shirt = shirts[grng.i(7)]; p.pants = grng.chance(0.6f) ? hexc(0x2a2f3a) : hexc(0x55504a);
        static const Col skins[] = {hexc(0xf1c8a6), hexc(0xe0b090), hexc(0xc89070)};
        p.skin = skins[grng.i(3)]; p.hair = grng.chance(0.5f) ? hexc(0x3a2a1a) : hexc(0x8a6a3a);
        p.h = grng.range(1.6f, 1.88f); p.speed = grng.range(1.1f, 1.6f);
        pedPickTarget(p);
        return;
    }
}
static bool pedCrossingNear(const V3& p, const V3& fwd, float range, float latW) {
    for (const Ped& q : PEDS) {
        if (!q.alive || q.state != 2) continue;
        V3 d = q.pos - p; float along = dot(d, fwd);
        if (along < 0 || along > range) continue;
        float lat = fabsf(d.x * fwd.z - d.z * fwd.x);
        if (lat < latW) return true;
    }
    return false;
}
static void updatePeds(float dt, const V3& center, bool active) {
    if (PEDS.empty()) PEDS.resize(26);
    for (Ped& p : PEDS) {
        if (!p.alive) { if (active && districtAt(center) == 0) spawnPed(p, center); continue; }
        if (dist2d(p.pos, center) > 190) { p.alive = false; continue; }
        if (p.state == 1) {
            // ждём красный для машин на пересекаемой улице
            p.wait += dt;
            int ax = lightAxisOf(EDGES[p.crossEdge]);
            if ((lightState(p.crossNode, ax) == 2 && lightRemaining(p.crossNode, ax) > 6) || p.wait > 40) p.state = 2;
            continue;
        }
        V3 d = p.target - p.pos; d.y = 0; float l = len(d);
        if (l < 0.3f) {
            if (p.state == 2) {
                // перешли — идём обратно по другой стороне
                p.state = 0; p.side = -p.side;
                const Edge& e = EDGES[p.edge];
                p.toNode = (p.toNode == e.a) ? e.b : e.a;
                pedPickTarget(p);
                continue;
            }
            const Edge& e = EDGES[p.edge];
            const Node& n = NODES[p.toNode];
            if (n.light && grng.chance(0.5f)) {
                // переход по «зебре» через текущую улицу
                p.crossEdge = p.edge; p.crossNode = p.toNode;
                V3 inw = (p.toNode == e.a) ? e.dir : -e.dir;
                V3 base = (p.toNode == e.a) ? e.pa : e.pb;
                V3 rr = rightOf(e.yaw);
                p.pos = base + inw * 2.2f + rr * (p.side * (e.halfW + 0.8f));
                p.target = base + inw * 2.2f + rr * (-p.side * (e.halfW + 0.8f));
                p.state = 1; p.wait = 0;
                continue;
            }
            // поворот за угол квартала: ближайший тротуар другой улицы
            float best = 1e9f; int be = -1, bs = 1;
            for (int ei : n.edges) {
                if (ei == p.edge) continue;
                const Edge& e2 = EDGES[ei];
                for (int sd = -1; sd <= 1; sd += 2) {
                    V3 sp = sidewalkPoint(e2, sd, e2.a == p.toNode ? -2.0f : e2.slen + 2.0f);
                    float dd = dist2d(sp, p.pos);
                    if (dd < best) { best = dd; be = ei; bs = sd; }
                }
            }
            if (be < 0 || best > 14 || EDGES[be].highway) { p.alive = false; continue; }
            p.edge = be; p.side = bs;
            const Edge& e2 = EDGES[be];
            p.toNode = (e2.a == p.toNode) ? e2.b : e2.a;
            pedPickTarget(p);
            continue;
        }
        V3 dn = d * (1 / l);
        p.yaw = atan2f(dn.x, dn.z);
        float sp = p.speed * (p.state == 2 ? 1.25f : 1);
        p.pos += dn * std::min(l, sp * dt);
        p.anim += dt * sp * 4.0f;
    }
}
static void drawPed(const Ped& p) {
    float dist = dist2d(p.pos, R3.pos);
    if (dist > 85 || !R3.sphereVisible(p.pos + V3(0, 1, 0), 1.2f)) return;
    float h = p.h, sw = sinf(p.anim) * (p.state == 1 ? 0 : 0.5f);
    V3 f = fwdOf(p.yaw), r = rightOf(p.yaw);
    float legH = h * 0.47f;
    for (int s = -1; s <= 1; s += 2) {
        V3 hip = p.pos + r * (s * 0.1f) + V3(0, legH, 0);
        float a = sw * s;
        V3 foot = hip + f * (sinf(a) * legH) - V3(0, cosf(a) * legH, 0);
        V3 mid = (hip + foot) * 0.5f;
        shadedBox(mid, V3(0.08f, legH * 0.5f, 0.09f), p.yaw + 0, p.pants);
    }
    shadedBox(p.pos + V3(0, legH + h * 0.15f, 0), V3(0.21f, h * 0.15f, 0.12f), p.yaw, p.shirt);
    for (int s = -1; s <= 1; s += 2) {
        float a = -sw * s * 0.8f;
        V3 sh = p.pos + r * (s * 0.27f) + V3(0, legH + h * 0.28f, 0);
        V3 hand = sh + f * (sinf(a) * h * 0.3f) - V3(0, cosf(a) * h * 0.3f, 0);
        shadedBox((sh + hand) * 0.5f, V3(0.06f, h * 0.15f, 0.06f), p.yaw, p.shirt * 0.9f);
    }
    shadedBox(p.pos + V3(0, h - 0.12f, 0), V3(0.11f, 0.12f, 0.11f), p.yaw, p.skin);
    shadedBox(p.pos + V3(0, h - 0.02f, -0.01f), V3(0.12f, 0.04f, 0.12f), p.yaw, p.hair);
    R3.groundSprite(p.pos + V3(0, 0.07f, 0), V3(0.35f, 0, 0), V3(0, 0, 0.35f), RGBA(0, 0, 0, (int)(90 * (1 - L.night * 0.6f))), 0, 0);
}

static void drawStatic() {
    V3 cam = R3.pos;
    int g0 = gridIdx(cam.x, cam.z); (void)g0;
    float range = std::min(L.fogEnd + 30, 380.f);
    int r = (int)(range / GRID_CS) + 1;
    int cgx = (int)((cam.x - GRID_X0) / GRID_CS), cgz = (int)((cam.z - GRID_Z0) / GRID_CS);
    static std::vector<int> seen; static int stamp = 0; static std::vector<int> mark;
    if (mark.size() != OBJS.size()) mark.assign(OBJS.size(), -1);
    stamp++;
    for (int gz = cgz - r; gz <= cgz + r; gz++) for (int gx = cgx - r; gx <= cgx + r; gx++) {
        if (gx < 0 || gz < 0 || gx >= GRID_NX || gz >= GRID_NZ) continue;
        for (int id : GRID[gz * GRID_NX + gx]) {
            if (mark[id] == stamp) continue;
            mark[id] = stamp;
            const SObj& o = OBJS[id];
            float d = dist2d(o.p, cam);
            float lim = range + o.radius;
            if (o.type == OB_TREE || o.type == OB_BUSH) lim = std::min(lim, 260.f);
            if (o.type == OB_BENCH) lim = 60;
            if (d > lim) continue;
            float cy = o.type == OB_BUILDING ? o.h.y * 0.5f : 2.0f;
            if (!R3.sphereVisible(o.p + V3(0, cy, 0), o.radius + cy + 1)) continue;
            switch (o.type) {
                case OB_BUILDING: drawBuilding(o, d); break;
                case OB_TREE: drawTree(o, d); break;
                case OB_BUSH: {
                    SObj t = o; t.h.y = o.h.y * 2.2f; drawTree(t, d);
                } break;
                case OB_LAMP: drawLamp(o, d); break;
                case OB_SIGN: if (d < 160) drawSign(o, d); break;
                case OB_TLIGHT: drawTrafficLight(o, d); break;
                case OB_BILLBOARD: drawBillboard(o, d); break;
                case OB_POST: drawPostBooth(o, d); break;
                case OB_KIOSK: drawKiosk(o, d); break;
                case OB_BENCH: drawBench(o, d); break;
                default: break;
            }
        }
    }
}

// столкновение круга со статикой: выталкивает, возвращает true при ударе
static bool collideStatic(V3& p, float rad, V3* normalOut) {
    int g = gridIdx(p.x, p.z); if (g < 0) return false;
    bool hit = false;
    for (int dz = -1; dz <= 1; dz++) for (int dx = -1; dx <= 1; dx++) {
        int gg = gridIdx(p.x + dx * GRID_CS, p.z + dz * GRID_CS); if (gg < 0) continue;
        for (int id : GRID[gg]) {
            const SObj& o = OBJS[id];
            if (!o.collide) continue;
            if (o.type == OB_BUILDING || o.type == OB_POST || o.type == OB_KIOSK) {
                float hx = o.h.x, hz = o.h.z;
                if (o.type != OB_BUILDING) { hx = std::max(o.h.x, o.h.z); hz = hx; }
                float qx = clampf(p.x, o.p.x - hx, o.p.x + hx), qz = clampf(p.z, o.p.z - hz, o.p.z + hz);
                float dx2 = p.x - qx, dz2 = p.z - qz, dd = sqrtf(dx2 * dx2 + dz2 * dz2);
                if (dd < rad) {
                    if (dd < 1e-4f) { dx2 = p.x - o.p.x; dz2 = p.z - o.p.z; dd = sqrtf(dx2 * dx2 + dz2 * dz2) + 1e-4f; }
                    V3 n(dx2 / dd, 0, dz2 / dd);
                    p += n * (rad - dd); hit = true; if (normalOut) *normalOut = n;
                }
            } else {
                float orad = (o.type == OB_TREE) ? 0.35f : (o.type == OB_BILLBOARD ? 0.4f : 0.2f);
                if (o.type == OB_BILLBOARD) {
                    V3 r = rightOf(o.yaw);
                    for (int s = -1; s <= 1; s += 2) {
                        V3 pp = o.p + r * (2.5f * s);
                        float dd = dist2d(p, pp);
                        if (dd < rad + orad) { V3 n = norm(V3(p.x - pp.x, 0, p.z - pp.z)); p += n * (rad + orad - dd); hit = true; if (normalOut) *normalOut = n; }
                    }
                    continue;
                }
                float dd = dist2d(p, o.p);
                if (dd < rad + orad) { V3 n = norm(V3(p.x - o.p.x, 0, p.z - o.p.z)); p += n * (rad + orad - dd); hit = true; if (normalOut) *normalOut = n; }
            }
        }
    }
    return hit;
}

// =====================================================================
//  Трафик и патрульная машина
// =====================================================================
enum ModelIds { MD_SEDAN = 0, MD_HATCH, MD_WAGON, MD_CROSS, MD_BIGSUV, MD_MINIVAN, MD_OLDJEEP, MD_PICKUP, MD_SPORTSEDAN, MD_OLDSEDAN, MD_SPORT1, MD_SPORT2, MD_HATCH2, MD_SEDAN2,
                MD_CLASSIC, MD_GAZEL, MD_TRUCK, MD_TRACTOR, MD_TRAILER };
enum CarMode { CMODE_DRIVE, CMODE_PULLOVER, CMODE_STOPPED, CMODE_FLEE, CMODE_LEAVE, CMODE_HELPER, CMODE_GONE, CMODE_BLOCK };

struct AICar {
    bool alive = false; int model = 0; Col paint, paint2; int plate = -1; std::string plateStr; int region = 66; bool taxi = false, police = false;
    int edge = 0, dir = 0, lane = 0; float s = 0;
    bool turning = false; float tt = 0, tlen = 1; V3 b0, b1, b2; int nEdge = 0, nDir = 0, nLane = 0;
    int planEdge = -1, planDir = 0;
    float lat = 0, latTarget = 0;
    V3 pos; float yaw = 0, speed = 0, desired = 1;
    float pitch = 0, roll = 0, pv = 0, rv = 0, spin = 0, steerVis = 0, lastSpeed = 0, lastYaw = 0;
    int mode = CMODE_DRIVE; float modeT = 0;
    bool brake = false; int blink = 0;
    bool speeder = false, redRunner = false;
    int viol = 0; float violSpeed = 0, violLimit = 0;   // 1 мелкое превышение, 2 превышение, 3 красный
    int caseIdx = -1; int demands = 0; float reactT = -1; bool ignoring = false;
    bool trailer = false; V3 tpos; float tyaw = 0;
    float pressure = 0; bool wanted = false;
    float yieldT = 0, stuck = 0, crashT = 0;
    float halfW = 0.9f, halfL = 2.3f;
    std::vector<V3> trail; int helperTarget = -1; int trailIdx = 0;
    float flashT = 0; bool caught = false; bool checked = false;
    float exhaustT = 0; uint32_t id = 0;
    bool vanish = false;
    float stuckD = 0, ghostT = 0;
};
static std::vector<AICar> CARS;
static uint32_t gCarIdCounter = 1;

struct Camera_ { int edge, dir; float s; };
static std::vector<Camera_> SPEEDCAMS;
struct Roadblock { bool active = false; int edge, dir; float s; V3 pos; float yaw; };
static Roadblock gBlock;

// --- частицы ---
struct Part { V3 p, v; float life = 0, maxLife = 1, size = 0.3f, grow = 0.6f; Col c; float a = 0.5f; bool add = false; };
static std::vector<Part> PARTS;
static void emitPart(const V3& p, const V3& v, float life, float size, const Col& c, float a, float grow = 0.6f, bool add = false) {
    if (PARTS.size() > 260) return;
    Part q; q.p = p; q.v = v; q.life = life; q.maxLife = life; q.size = size; q.c = c; q.a = a; q.grow = grow; q.add = add;
    PARTS.push_back(q);
}
static void updateParts(float dt) {
    for (size_t i = 0; i < PARTS.size();) {
        Part& p = PARTS[i]; p.life -= dt;
        if (p.life <= 0) { PARTS[i] = PARTS.back(); PARTS.pop_back(); continue; }
        p.p += p.v * dt; p.v *= (1 - 1.5f * dt); p.v.y += 0.25f * dt; p.size += p.grow * dt;
        i++;
    }
}
static void drawParts() {
    for (const Part& p : PARTS) {
        float k = p.life / p.maxLife;
        float d = dist2d(p.p, R3.pos); if (d > 80) continue;
        Col c = litFlat(p.c);
        if (p.add) R3.billboard(p.p, p.size, p.size, toSDL(p.c * k), 0, 3, 0);
        else R3.billboard(p.p, p.size, p.size, toSDL(lerpc(c, L.fog, fogFactor(d)), p.a * k), 0, 2, 0);
    }
}

// ---------------------------------------------------------------------
//  Патрульные машины и прокачка
// ---------------------------------------------------------------------
struct PatrolSpec { const char* name; int model; int price; int rank; float vmax, accel, brake, grip, mass; const char* desc; };
static const PatrolSpec PATROL[4] = {
    {"ЖИГУЛИ", MD_CLASSIC, 0, 0, 175, 4.8f, 10.0f, 0.95f, 1.0f, "СТАРАЯ ДОБРАЯ «СЕМЁРКА» С ФОРСИРОВАННЫМ МОТОРОМ."},
    {"СЕДАН", MD_SEDAN, 60000, 2, 230, 6.3f, 11.5f, 1.05f, 1.2f, "СОВРЕМЕННЫЙ СЕДАН. КОМФОРТ И ДИНАМИКА."},
    {"ВНЕДОРОЖНИК", MD_BIGSUV, 140000, 4, 220, 6.0f, 11.0f, 1.0f, 2.2f, "ТЯЖЁЛЫЙ. ЛЕГКО ПРИЖИМАЕТ БЕГЛЕЦОВ."},
    {"МОЩНЫЙ СЕДАН", MD_SPORTSEDAN, 190000, 6, 290, 9.0f, 13.5f, 1.18f, 1.3f, "ПЕРЕХВАТЧИК. ОТ НЕГО НЕ УЙТИ."}};
enum Upg { U_ENGINE, U_BRAKES, U_TIRES, U_LIGHTS, U_RADAR, U_CAMS, U_COUNT };
static const char* UPG_NAMES[U_COUNT] = {"ДВИГАТЕЛЬ", "ТОРМОЗА", "ШИНЫ", "МИГАЛКИ", "РАДАР", "КАМЕРЫ"};
static const char* UPG_DESC[U_COUNT] = {"СКОРОСТЬ И РАЗГОН", "КОРОЧЕ ТОРМОЗНОЙ ПУТЬ", "ЛУЧШЕ СЦЕПЛЕНИЕ", "ВОДИТЕЛИ ЧАЩЕ ПОДЧИНЯЮТСЯ", "ДАЛЬШЕ ЗАМЕР СКОРОСТИ", "БОЛЬШЕ КАМЕР НА КАРТЕ"};
static const int UPG_PRICE[U_COUNT][3] = {{15000, 35000, 70000}, {8000, 18000, 35000}, {10000, 22000, 40000}, {6000, 15000, 30000}, {12000, 25000, 50000}, {20000, 40000, 80000}};
static const int CAM_SLOTS[4] = {2, 3, 5, 8};
static const float RADAR_RANGE[4] = {70, 100, 135, 170};
static const int CAMERA_PRICE = 8000;

struct Save {
    int money = 3000, rep = 0, day = 1, carOwned = 1, car = 0; int upg[U_COUNT] = {0, 0, 0, 0, 0, 0};
    int complaints = 0, totalChecks = 0, totalFines = 0, totalDetain = 0, totalChases = 0, totalCorrect = 0, totalWrong = 0;
    int dateDays = 0; bool sound = true; int camView = 0; int lastDistrict = 0;
    bool tutorialShown = false;
};
static Save SV;

// ---------------------------------------------------------------------
//  Игрок
// ---------------------------------------------------------------------
struct PlayerCar {
    int spec = 0; V3 pos; float yaw = 0, speed = 0, steer = 0, yawRate = 0;
    float pitch = 0, roll = 0, pv = 0, rv = 0, spin = 0;
    bool siren = false, flash = false, brakeLight = false, reverse = false, offroad = false, sliding = false;
    float lastSpeed = 0, rpm = 0, load = 0, exhaustT = 0, hitCooldown = 0, limitKmh = 60;
    float halfW = 0.85f, halfL = 2.1f;
};
static PlayerCar PL;
struct Input { float steer = 0, throttle = 0, brake = 0; };

static float specVmax() { const PatrolSpec& s = PATROL[SV.car]; return s.vmax / 3.6f * (1 + 0.07f * SV.upg[U_ENGINE]); }
static float specAccel() { const PatrolSpec& s = PATROL[SV.car]; return s.accel * (1 + 0.15f * SV.upg[U_ENGINE]); }
static float specBrake() { const PatrolSpec& s = PATROL[SV.car]; return s.brake * (1 + 0.15f * SV.upg[U_BRAKES]); }
static float specGrip() { const PatrolSpec& s = PATROL[SV.car]; return s.grip * (1 + 0.08f * SV.upg[U_TIRES]) * (L.rain > 0.3f ? 0.82f : 1.0f); }

static void playerSetCar() {
    PL.spec = SV.car;
    const CarModel& M = CM[PATROL[SV.car].model];
    PL.halfW = M.halfW; PL.halfL = M.halfL;
}

// круги для столкновений (локальные смещения по оси машины)
static int carCircles(const V3& pos, float yaw, float halfW, float halfL, V3* out) {
    V3 f = fwdOf(yaw);
    float r = halfW; float span = std::max(0.0f, halfL - r);
    int n = span > 2.2f ? 3 : 2;
    if (span > 5) n = 5;
    for (int i = 0; i < n; i++) { float t = n == 1 ? 0 : (i / (float)(n - 1)) * 2 - 1; out[i] = pos + f * (span * t); }
    return n;
}

static void sfxCrash(float strength) { sfx(SFX_CRASH, clampf(strength, 0.2f, 1.0f)); }

static float gCrashFlag = 0;   // для геймплея: последний удар
static int gLastHitCar = -1;

static void updatePlayer(float dt, const Input& in) {
    const CarModel& M = CM[PATROL[PL.spec].model];
    float vmax = specVmax(), acc = specAccel(), brk = specBrake(), grip = specGrip();
    float lim = 60;
    PL.offroad = !onRoadSurface(PL.pos, &lim);
    PL.limitKmh = lim;
    if (PL.siren) vmax *= 1.08f;          // режим преследования
    if (PL.offroad) vmax *= 0.72f;
    float thr = in.throttle, br = in.brake;
    PL.reverse = false;
    float sp = std::max(PL.speed, 0.f);
    if (thr > 0.01f) {
        if (PL.speed < -0.3f) PL.speed = approach(PL.speed, 0, brk * dt * thr);
        else {
            // тяга падает квадратично — машина уверенно набирает высокую скорость
            float k = 1 - (sp / vmax) * (sp / vmax);
            if (k < 0) k = 0;
            PL.speed += acc * k * thr * dt * (sp < 12 ? 1.25f : 1.0f);
        }
    }
    if (br > 0.01f) {
        if (PL.speed > 0.3f) PL.speed = std::max(0.f, PL.speed - brk * br * dt);
        else { PL.speed = std::max(PL.speed - 4.0f * dt * br, -9.0f); PL.reverse = true; }
    }
    if (thr <= 0.01f && br <= 0.01f) {
        float drag = (0.4f + fabsf(PL.speed) * 0.012f) * dt;
        PL.speed = approach(PL.speed, 0, drag);
    }
    if (PL.offroad) PL.speed = approach(PL.speed, 0, fabsf(PL.speed) * 0.18f * dt);
    if (PL.speed > vmax) PL.speed = approach(PL.speed, vmax, 3 * dt);
    float av = fabsf(PL.speed);
    // руль: быстрый отклик на малой скорости, мягче на большой, быстрый возврат в ноль
    float maxSteer = 0.62f / (1 + av * 0.028f);
    float tgt = in.steer * maxSteer;
    float rate = fabsf(tgt) < fabsf(PL.steer) || tgt * PL.steer < 0 ? 6.0f : (3.6f - std::min(av, 35.f) * 0.045f);
    PL.steer = approach(PL.steer, tgt, rate * dt);
    float yr = PL.speed * tanf(PL.steer) / M.wheelbase;
    // аркадное сцепление: держит ~1.3–1.7 g
    float maxLat = 9.81f * grip * 1.45f;
    PL.sliding = false;
    if (fabsf(yr * PL.speed) > maxLat && av > 1) {
        float lim2 = maxLat / av;
        PL.sliding = fabsf(yr) > lim2 * 1.15f && av > 12;
        yr = (yr > 0 ? 1 : -1) * lim2;
    }
    // сглаживание скорости поворота — без рывков от цифровых кнопок
    PL.yawRate = approach(PL.yawRate, yr, 9.0f * dt * std::max(1.0f, fabsf(yr - PL.yawRate)));
    yr = PL.yawRate;
    PL.yaw = wrapAngle(PL.yaw + yr * dt);
    // помощь удержания полосы: без руля машина мягко выравнивается вдоль дороги
    if (fabsf(in.steer) < 0.01f && av > 6 && !PL.offroad) {
        RoadHit h = nearestRoad(PL.pos);
        if (h.edge >= 0 && !h.onNode) {
            float ry = EDGES[h.edge].yaw;
            float e1 = wrapAngle(ry - PL.yaw), e2 = wrapAngle(ry + PI - PL.yaw);
            float err = fabsf(e1) < fabsf(e2) ? e1 : e2;
            if (fabsf(err) < 0.3f) PL.yaw = wrapAngle(PL.yaw + clampf(err, -1, 1) * 1.6f * dt);
        }
    }
    PL.pos += fwdOf(PL.yaw) * (PL.speed * dt);
    // столкновения со статикой
    V3 cs[5]; int n = carCircles(PL.pos, PL.yaw, PL.halfW, PL.halfL, cs);
    PL.hitCooldown -= dt;
    for (int i = 0; i < n; i++) {
        V3 p = cs[i], nrm;
        if (collideStatic(p, PL.halfW, &nrm)) {
            V3 delta = p - cs[i];
            PL.pos += delta;
            float into = -dot(fwdOf(PL.yaw) * PL.speed, nrm);
            if (into > 0.5f) {
                PL.speed *= 0.35f;
                if (into > 4 && PL.hitCooldown <= 0) { sfxCrash(into / 15); PL.hitCooldown = 0.5f; gCrashFlag = into; }
            }
            for (int k = 0; k < n; k++) cs[k] += delta;
        }
    }
    // кузов
    float al = (PL.speed - PL.lastSpeed) / std::max(dt, 1e-3f);
    PL.lastSpeed = PL.speed;
    float pt = clampf(-al * 0.007f, -0.05f, 0.06f), rt = clampf(yr * PL.speed * 0.011f, -0.07f, 0.07f);
    PL.pv += ((pt - PL.pitch) * 70 - PL.pv * 9) * dt; PL.pitch += PL.pv * dt;
    PL.rv += ((rt - PL.roll) * 60 - PL.rv * 8) * dt; PL.roll += PL.rv * dt;
    float wr = M.wheels.empty() ? 0.3f : M.wheels[0].r;
    PL.spin += PL.speed * dt / wr;
    PL.brakeLight = br > 0.05f && PL.speed > 0.2f;
    // двигатель: условные передачи
    float kmh = fabsf(PL.speed) * 3.6f;
    float gearTop[5] = {25, 50, 80, 115, 400};
    int g = 0; while (g < 4 && kmh > gearTop[g]) g++;
    float lo = g == 0 ? 0 : gearTop[g - 1], hi = gearTop[g] > 300 ? lo + 120 : gearTop[g];
    float rpmT = clampf(0.15f + 0.85f * (kmh - lo) / (hi - lo), 0.12f, 1.0f);
    if (kmh < 2) rpmT = 0.12f + thr * 0.5f;
    PL.rpm = approach(PL.rpm, rpmT, 3 * dt);
    PL.load = approach(PL.load, thr, 4 * dt);
    // дым
    PL.exhaustT -= dt;
    if (PL.exhaustT <= 0) {
        PL.exhaustT = 0.08f - thr * 0.04f;
        Mat3 rot = rotYPR(PL.yaw, 0, 0);
        V3 ep = mul(rot, M.exhaust) + PL.pos;
        emitPart(ep, fwdOf(PL.yaw) * (-1.2f) + V3(grng.range(-0.2f, 0.2f), 0.3f, 0), 0.9f + thr, 0.12f, Col(0.6f, 0.6f, 0.62f), 0.25f + thr * 0.25f, 0.7f);
    }
    if (PL.sliding && fabsf(PL.speed) > 6 && grng.chance(0.5f)) {
        Mat3 rot = rotYPR(PL.yaw, 0, 0);
        for (size_t i = 2; i < M.wheels.size() && i < 4; i++) emitPart(mul(rot, M.wheels[i].p) + PL.pos - V3(0, 0.2f, 0), V3(0, 0.6f, 0), 1.2f, 0.25f, Col(0.85f, 0.85f, 0.85f), 0.35f, 1.0f);
    }
}

// ---------------------------------------------------------------------
//  ИИ-машины
// ---------------------------------------------------------------------
static float edgeLimitMs(const Edge& e) { return e.limitKmh / 3.6f; }

static void planNext(AICar& c) {
    const Edge& e = EDGES[c.edge];
    int node = edgeEndNode(e, c.dir);
    const Node& n = NODES[node];
    std::vector<int> cand;
    for (int ei : n.edges) if (ei != c.edge) cand.push_back(ei);
    // фуры и грузовики — по трассе
    if (c.trailer || c.model == MD_TRUCK) {
        std::vector<int> hw; for (int ei : cand) if (EDGES[ei].highway) hw.push_back(ei);
        if (!hw.empty()) cand = hw;
    }
    int ne;
    if (cand.empty()) ne = c.edge;
    else {
        // предпочитаем ехать прямо
        int straight = -1; float best = -2;
        for (int ei : cand) {
            const Edge& e2 = EDGES[ei]; V3 d2 = (e2.a == node) ? e2.dir : -e2.dir;
            V3 d1 = c.dir == 0 ? e.dir : -e.dir;
            float dd = dot(d1, d2); if (dd > best) { best = dd; straight = ei; }
        }
        if (best > 0.9f && grng.chance(0.55f)) ne = straight; else ne = cand[grng.i((int)cand.size())];
    }
    c.planEdge = ne;
    c.planDir = (EDGES[ne].a == node) ? 0 : 1;
    if (ne == c.edge) c.planDir = 1 - c.dir;
}

// -1 налево, 0 прямо, 1 направо
static int turnSign(const Edge& e, int dir, int ne, int ndir) {
    const Edge& e2 = EDGES[ne];
    V3 f0 = dir == 0 ? e.dir : -e.dir, f2 = ndir == 0 ? e2.dir : -e2.dir;
    float cr = f0.x * f2.z - f0.z * f2.x, dd = dot(f0, f2);
    if (dd > 0.7f) return 0;
    return cr < 0 ? 1 : -1;
}
static void beginTurn(AICar& c) {
    const Edge& e = EDGES[c.edge];
    if (c.planEdge < 0) planNext(c);
    // повороты — только из крайней полосы; иначе едем прямо, если можно
    if (e.lanes > 1 && c.mode != CMODE_FLEE && c.planEdge != c.edge) {
        int ts = turnSign(e, c.dir, c.planEdge, c.planDir);
        int need = ts == 1 ? e.lanes - 1 : (ts == -1 ? 0 : c.lane);
        if (need != c.lane) {
            int node = edgeEndNode(e, c.dir);
            for (int ei : NODES[node].edges) {
                if (ei == c.edge) continue;
                int nd = (EDGES[ei].a == node) ? 0 : 1;
                if (turnSign(e, c.dir, ei, nd) == 0) { c.planEdge = ei; c.planDir = nd; break; }
            }
        }
    }
    c.nEdge = c.planEdge; c.nDir = c.planDir;
    const Edge& e2 = EDGES[c.nEdge];
    c.nLane = std::min(c.lane, e2.lanes - 1);
    V3 p0 = lanePos(e, c.dir, c.lane, e.slen, 0);
    V3 p2 = lanePos(e2, c.nDir, c.nLane, 0, 0);
    V3 f0 = c.dir == 0 ? e.dir : -e.dir, f2 = c.nDir == 0 ? e2.dir : -e2.dir;
    float den = f0.x * f2.z - f0.z * f2.x;
    V3 p1;
    if (fabsf(den) < 0.2f) p1 = (p0 + p2) * 0.5f;
    else {
        V3 d = p2 - p0;
        float t = (d.x * f2.z - d.z * f2.x) / den;
        p1 = p0 + f0 * t;
        if (t < 0 || t > 80) p1 = (p0 + p2) * 0.5f;
    }
    c.b0 = p0; c.b1 = p1; c.b2 = p2;
    c.tlen = std::max(2.0f, (len(p1 - p0) + len(p2 - p1) + len(p2 - p0)) * 0.5f);
    c.turning = true; c.tt = 0;
    // поворотник
    float cr = f0.x * f2.z - f0.z * f2.x;
    c.blink = (c.mode == CMODE_DRIVE && fabsf(cr) > 0.3f) ? (cr < 0 ? 1 : -1) : (c.mode == CMODE_STOPPED ? 2 : 0);
}

static void carComputePose(AICar& c) {
    if (c.turning) {
        float t = clampf(c.tt, 0, 1);
        V3 p = c.b0 * ((1 - t) * (1 - t)) + c.b1 * (2 * t * (1 - t)) + c.b2 * (t * t);
        V3 tg = (c.b1 - c.b0) * (2 * (1 - t)) + (c.b2 - c.b1) * (2 * t);
        if (len(tg) < 1e-3f) tg = c.b2 - c.b0;
        float y = atan2f(tg.x, tg.z);
        c.pos = p + rightOf(y) * c.lat; c.yaw = y;
    } else {
        const Edge& e = EDGES[c.edge];
        V3 st, f, r; float y; laneFrame(e, c.dir, st, f, r, y);
        c.pos = st + f * c.s + r * (laneOffset(e, c.lane) + c.lat);
        // плавный разворот при смене бокового смещения
        float dl = c.latTarget - c.lat;
        c.yaw = wrapAngle(y + clampf(dl * 0.12f, -0.25f, 0.25f) * (c.speed > 1 ? 1 : c.speed));
    }
}

static int pickTrafficModel(bool highway, bool& trailer, bool& taxi, Col& paint, Col& paint2) {
    static const Col colors[] = {hexc(0xf0f0f0), hexc(0x151515), hexc(0xa8aab0), hexc(0x6a6d72), hexc(0x1d2f5a), hexc(0x7a1418), hexc(0x2a4a2e), hexc(0xc9b48a),
                                 hexc(0x3a3f45), hexc(0x8a8f96), hexc(0xd8d8d0), hexc(0x274a78), hexc(0x5a2d1a), hexc(0xb02020)};
    trailer = false; taxi = false;
    paint = colors[grng.i(14)];
    paint2 = grng.chance(0.5f) ? hexc(0x2a5aa0) : (grng.chance(0.5f) ? hexc(0xe8e8e8) : hexc(0x2a6a3a));
    float r = grng.f();
    if (highway) {
        if (r < 0.12f) { trailer = true; paint = grng.chance(0.5f) ? hexc(0xd02020) : hexc(0xf0f0f0); return MD_TRACTOR; }
        if (r < 0.22f) { paint = grng.chance(0.5f) ? hexc(0x2060b0) : hexc(0xe0e0e0); return MD_TRUCK; }
        if (r < 0.27f) return MD_GAZEL;
    } else {
        if (r < 0.08f) { paint = hexc(0xf2d21a); return MD_GAZEL; }
        if (r < 0.13f) { taxi = true; paint = hexc(0xf6c40f); return grng.chance(0.5f) ? MD_SEDAN : MD_SEDAN2; }
        if (r < 0.16f) { paint = hexc(0xe0e0e0); return MD_TRUCK; }
    }
    static const int pool[] = {MD_SEDAN, MD_HATCH, MD_WAGON, MD_CROSS, MD_BIGSUV, MD_MINIVAN, MD_OLDJEEP, MD_PICKUP, MD_SPORTSEDAN, MD_OLDSEDAN, MD_SPORT1, MD_SPORT2, MD_HATCH2, MD_SEDAN2,
                               MD_CLASSIC, MD_CLASSIC, MD_CLASSIC, MD_SEDAN, MD_HATCH, MD_SEDAN2, MD_CROSS, MD_HATCH2};
    int m = pool[grng.i(sizeof(pool) / sizeof(pool[0]))];
    if (m == MD_CLASSIC) { static const Col lada[] = {hexc(0xe8e4d8), hexc(0x8a1a1a), hexc(0x2a4a7a), hexc(0x4a6a3a), hexc(0xc8b070), hexc(0x303030)}; paint = lada[grng.i(6)]; }
    return m;
}

static int gPlateCounter = 4;
static int allocPlate(std::string& str, int& region, bool police = false) {
    int slot = gPlateCounter; gPlateCounter++; if (gPlateCounter >= PLATE_SLOTS) gPlateCounter = 4;
    if (police) { str = fmt("%s%04d", "А", grng.i(9000) + 1000); region = 66; }
    else str = randomPlate(grng, region);
    platePixels(slot, str, region, police);
    return slot;
}

static void initCarOnLane(AICar& c, int edge, int dir, int lane, float s) {
    c.edge = edge; c.dir = dir; c.lane = lane; c.s = s; c.turning = false; c.lat = 0; c.latTarget = 0;
    c.planEdge = -1; planNext(c);
    carComputePose(c);
    c.lastYaw = c.yaw;
    if (c.trailer) { c.tyaw = c.yaw; V3 hitch = c.pos + fwdOf(c.yaw) * CM[MD_TRACTOR].hitchZ; c.tpos = hitch - fwdOf(c.tyaw) * CM[MD_TRAILER].kingpinZ; }
}

static bool laneFree(int edge, int dir, int lane, float s, float range) {
    for (const AICar& o : CARS) {
        if (!o.alive || o.turning || o.edge != edge || o.dir != dir || o.lane != lane) continue;
        if (fabsf(o.s - s) < range) return false;
    }
    return true;
}

static int spawnTrafficCar(const V3& center, float minD, float maxD, bool avoidView) {
    for (int tries = 0; tries < 12; tries++) {
        int ei = grng.i((int)EDGES.size());
        const Edge& e = EDGES[ei];
        float near0 = clampf(dot(center - e.pa, e.dir), 0, e.slen);
        float s = clampf(near0 + grng.range(-maxD, maxD), 2, e.slen - 2);
        int dir = grng.i(2), lane = grng.i(e.lanes);
        float sl = dir == 0 ? s : e.slen - s;
        V3 p = lanePos(e, dir, lane, sl, 0);
        float d = dist2d(p, center);
        if (d < minD || d > maxD) continue;
        if (avoidView && d < 220) {
            V3 v = R3.toView(p); if (v.z > 0 && fabsf(v.x) < v.z * 1.2f) continue;
        }
        if (!laneFree(ei, dir, lane, sl, 18)) continue;
        bool clear = true; for (const AICar& o : CARS) if (o.alive && (dist2d(o.pos, p) < 14 || (o.trailer && dist2d(o.tpos, p) < 14))) { clear = false; break; }
        if (!clear) continue;
        if (dist2d(p, PL.pos) < 25) continue;
        AICar c; c.alive = true; c.id = gCarIdCounter++;
        c.model = pickTrafficModel(e.highway, c.trailer, c.taxi, c.paint, c.paint2);
        if (!e.highway && (c.trailer)) continue;
        if (c.trailer && lane == 0 && e.lanes > 1) lane = 1;
        const CarModel& M = CM[c.model]; c.halfW = M.halfW; c.halfL = M.halfL;
        bool heavy = c.trailer || c.model == MD_TRUCK;
        c.speeder = !heavy && grng.chance(e.highway ? 0.16f : 0.08f);
        c.redRunner = !heavy && grng.chance(0.06f);
        c.desired = c.speeder ? grng.range(1.3f, 1.65f) : grng.range(0.82f, 1.1f);
        if (heavy) c.desired = grng.range(0.75f, 0.9f);
        c.plate = allocPlate(c.plateStr, c.region);
        initCarOnLane(c, ei, dir, lane, sl);
        c.speed = edgeLimitMs(e) * c.desired * 0.8f;
        // занять свободный слот
        for (size_t i = 0; i < CARS.size(); i++) if (!CARS[i].alive) { CARS[i] = c; return (int)i; }
        CARS.push_back(c); return (int)CARS.size() - 1;
    }
    return -1;
}

// ближайшее препятствие впереди: возвращает зазор и скорость лидера
static void findLeader(const AICar& c, int selfIdx, float& gap, float& vLead) {
    gap = 1e9f; vLead = 0;
    V3 f = fwdOf(c.yaw), r = rightOf(c.yaw);
    float look = std::max(55.0f, c.speed * 3.2f + 15);
    for (size_t i = 0; i < CARS.size(); i++) {
        if ((int)i == selfIdx || c.ghostT > 0) continue;
        const AICar& o = CARS[i]; if (!o.alive || o.mode == CMODE_GONE) continue;
        // слияние в одну полосу после поворота: приоритет у того, кто ближе к выходу
        bool mergeLeader = false;
        if (c.turning && o.turning && o.nEdge == c.nEdge && o.nDir == c.nDir && o.nLane == c.nLane && o.edge != c.edge) {
            mergeLeader = (1 - o.tt) * o.tlen < (1 - c.tt) * c.tlen;
            if (!mergeLeader) continue;
        }
        if (c.turning && !o.turning && o.edge == c.nEdge && o.dir == c.nDir && o.lane == c.nLane && o.s < 25) mergeLeader = true;
        if (mergeLeader) {
            float g = dist2d(o.pos, c.pos) - c.halfL - o.halfL;
            if (g < gap) { gap = g; vLead = o.speed; }
            continue;
        }
        // в перекрёстке пересекающиеся траектории не считаем лидерами (иначе — взаимная блокировка)
        if ((c.turning || o.turning) && dot(fwdOf(o.yaw), f) < 0.35f) continue;
        V3 objs[2] = {o.pos, o.tpos}; int no = o.trailer ? 2 : 1;
        for (int k = 0; k < no; k++) {
            V3 d = objs[k] - c.pos; float along = dot(d, f);
            float ohl = k == 0 ? o.halfL : CM[MD_TRAILER].halfL;
            if (along < -ohl || along > look + ohl) continue;
            float lat = dot(d, r);
            float thr = c.halfW + o.halfW * 0.9f + 0.15f;
            bool sameLane = k == 0 && !c.turning && !o.turning && o.edge == c.edge && o.dir == c.dir && o.lane == c.lane;
            if (fabsf(lat) > thr && !sameLane) continue;
            // встречные на повороте не считаем
            if (dot(fwdOf(k == 0 ? o.yaw : o.tyaw), f) < -0.3f && fabsf(lat) > 1.0f) continue;
            float g = along - c.halfL - ohl;
            if (g < gap) { gap = g; vLead = std::max(0.f, o.speed * dot(fwdOf(o.yaw), f)); }
        }
    }
    // игрок
    if (c.mode != CMODE_HELPER) {
        V3 d = PL.pos - c.pos; float along = dot(d, f), lat = dot(d, r);
        if (along > -PL.halfL && along < look && fabsf(lat) < c.halfW + PL.halfW * 0.9f + 0.2f) {
            float g = along - c.halfL - PL.halfL;
            if (g < gap) { gap = g; vLead = std::max(0.f, PL.speed * dot(fwdOf(PL.yaw), f)); }
        }
    }
    // блокпост
    if (gBlock.active) {
        V3 d = gBlock.pos - c.pos; float along = dot(d, f), lat = dot(d, r);
        if (along > 0 && along < 90 && fabsf(lat) < 9) { float g = along - c.halfL - 3; if (g < gap) { gap = g; vLead = 0; } }
    }
}

static void carAccel(AICar& c, float v0, float gap, float vl, float dt, float amax, float bcomf) {
    float v = c.speed;
    if (v0 < 0.1f) v0 = 0.1f;
    float sStar = 2.0f + v * 1.1f + v * (v - vl) / (2 * sqrtf(amax * bcomf));
    if (sStar < 2) sStar = 2;
    float g = std::max(gap, 0.1f);
    float acc = amax * (1 - powf(std::min(v / v0, 2.0f), 4) - (sStar / g) * (sStar / g));
    acc = clampf(acc, -9.5f, amax);
    c.speed = std::max(0.f, v + acc * dt);
    c.brake = acc < -1.2f || (c.speed < 0.3f && v0 > 0.5f);
}

static void tryLaneChange(AICar& c, int newLane) {
    const Edge& e = EDGES[c.edge];
    if (newLane < 0 || newLane >= e.lanes || newLane == c.lane || c.turning) return;
    if (c.s > e.slen - 25) return;
    if (c.speed < 3 && c.mode != CMODE_PULLOVER) return;
    // проверка: свободна ли целевая полоса
    for (const AICar& o : CARS) {
        if (!o.alive || &o == &c || o.turning || o.edge != c.edge || o.dir != c.dir || o.lane != newLane) continue;
        float ds = o.s - c.s;
        if (ds > -12 - std::max(0.f, o.speed - c.speed) * 2.5f - o.speed * 0.5f && ds < 22) return;
    }
    V3 d = PL.pos - c.pos; float along = dot(d, fwdOf(c.yaw)); float lat = dot(d, rightOf(c.yaw));
    float wantLat = (newLane - c.lane) * e.laneW;
    if (fabsf(lat - wantLat) < 2.5f && along > -12 && along < 22) return;
    c.lat += (c.lane - newLane) * e.laneW;
    c.lane = newLane;
}

static float shoulderLat(const AICar& c) {
    const Edge& e = EDGES[c.edge];
    float off = laneOffset(e, c.lane);
    if (e.highway) return e.halfW + 1.6f - off;
    return e.halfW - off - c.halfW - 0.15f;
}

static int gChaseCar = -1;
static float gPlayerSirenRange = 70;

static void updateAICar(AICar& c, int idx, float dt) {
    c.modeT += dt;
    if (c.crashT > 0) c.crashT -= dt;
    const Edge& e = EDGES[c.edge];
    float lim = edgeLimitMs(e);
    float v0 = lim * c.desired;
    float amax = (c.trailer || c.model == MD_TRUCK) ? 1.1f : 2.2f, bcomf = 3.5f;
    bool ignoreLights = false;
    if (c.mode == CMODE_HELPER) {
        // коллега едет по следу беглеца
        if (c.helperTarget < 0 || c.helperTarget >= (int)CARS.size() || !CARS[c.helperTarget].alive) { c.mode = CMODE_GONE; return; }
        AICar& t = CARS[c.helperTarget];
        V3 goal = c.trailIdx < (int)t.trail.size() ? t.trail[c.trailIdx] : t.pos;
        V3 d = goal - c.pos; d.y = 0; float l = len(d);
        if (l < 3 && c.trailIdx < (int)t.trail.size()) { c.trailIdx++; }
        float dt2 = dist2d(c.pos, t.pos);
        float vt = dt2 > 18 ? std::max(t.speed * 1.12f + 4, 12.f) : t.speed * 0.95f;
        c.speed = approach(c.speed, std::min(vt, 52.f), 6 * dt);
        if (l > 0.1f) {
            float ty = atan2f(d.x, d.z);
            c.yaw = wrapAngle(c.yaw + clampf(wrapAngle(ty - c.yaw), -2.5f * dt, 2.5f * dt));
        }
        c.pos += fwdOf(c.yaw) * (c.speed * dt);
        if (dt2 < 16 && t.mode == CMODE_FLEE) t.pressure += dt * 0.55f;
        c.spin += c.speed * dt / 0.33f;
        return;
    }
    if (c.mode == CMODE_BLOCK) { c.speed = 0; return; }
    if (c.mode == CMODE_STOPPED) {
        c.speed = approach(c.speed, 0, 6 * dt); c.blink = 2; c.brake = true;
        c.lat = approach(c.lat, c.latTarget, 1.0f * dt);
        carComputePose(c);
        return;
    }
    if (c.mode == CMODE_FLEE) {
        v0 = lim * (e.highway ? 1.75f : 1.6f); amax = 3.6f; bcomf = 6; ignoreLights = true;
        if (c.pressure > 0) c.pressure -= dt * 0.08f;
        if (c.trail.empty() || dist2d(c.trail.back(), c.pos) > 4) { c.trail.push_back(c.pos); if (c.trail.size() > 800) c.trail.erase(c.trail.begin(), c.trail.begin() + 100); }
    }
    if (c.mode == CMODE_PULLOVER) {
        // в крайнюю правую полосу и на обочину
        if (!c.turning && c.lane < e.lanes - 1) tryLaneChange(c, e.lanes - 1);
        if (!c.turning) c.latTarget = shoulderLat(c);
        c.blink = 1;
        bool there = !c.turning && fabsf(c.lat - c.latTarget) < 0.6f;
        v0 = there ? 0.0f : std::max(4.0f, std::min(v0, 9.0f));
        if (there && c.speed < 0.25f) { c.mode = CMODE_STOPPED; c.modeT = 0; c.speed = 0; c.blink = 2; }
    }
    if (c.mode == CMODE_LEAVE) {
        c.latTarget = 0; c.blink = c.modeT < 3 ? -1 : 0;
        if (c.modeT > 4 && fabsf(c.lat) < 0.3f) { c.mode = CMODE_DRIVE; }
        if (c.modeT < 1.5f) v0 = 0;
    }
    if (c.mode == CMODE_DRIVE) {
        // уступить дорогу машине с сиреной
        if (PL.siren && gChaseCar != idx) {
            V3 d = c.pos - PL.pos; float along = dot(d, fwdOf(PL.yaw));
            if (along > 0 && along < gPlayerSirenRange && fabsf(dot(d, rightOf(PL.yaw))) < 8 && dot(fwdOf(c.yaw), fwdOf(PL.yaw)) > 0.6f) c.yieldT = 2.5f;
        }
        if (c.yieldT > 0) {
            c.yieldT -= dt;
            if (!c.turning) { if (c.lane < e.lanes - 1) tryLaneChange(c, e.lanes - 1); c.latTarget = shoulderLat(c) * 0.75f; }
            v0 *= 0.45f; c.blink = 1;
            if (c.yieldT <= 0) { c.latTarget = 0; c.blink = 0; }
        } else c.latTarget = 0;
        // камеры: притормаживают
        for (const Camera_& cam : SPEEDCAMS) {
            if (cam.edge != c.edge || cam.dir != c.dir || c.turning) continue;
            float ds = cam.s - c.s;
            if (ds > -5 && ds < 160 && !(c.speeder && (c.id % 3 == 0))) v0 = std::min(v0, lim * 0.96f);
            if (ds < 0 && ds > -c.speed * dt - 0.01f && c.speed > lim + 20 / 3.6f) c.flashT = 0.25f;
        }
    }
    // светофор
    float gap, vl; findLeader(c, idx, gap, vl);
    bool redStop = false;
    if (!c.turning && !ignoreLights) {
        int node = edgeEndNode(e, c.dir);
        float distEnd = e.slen - c.s;
        if (NODES[node].light && distEnd < std::max(60.f, c.speed * c.speed / 5 + 20) && distEnd > 3.5f) {
            int st = lightState(node, lightAxisOf(e));
            float stopAt = distEnd - 4.8f;
            bool mustStop = st == 2 || (st == 1 && stopAt > c.speed * c.speed / (2 * 4.0f));
            if (mustStop && c.redRunner && c.mode == CMODE_DRIVE) mustStop = false;
            if (mustStop) { float g = stopAt - c.halfL * 0.0f - 0.5f; redStop = true; if (g < gap) { gap = g; vl = 0; } }
        }
    }
    // пешеходы на переходе
    if (!ignoreLights && !PEDS.empty() && pedCrossingNear(c.pos, fwdOf(c.yaw), c.halfL + 10, 3.0f)) { float g = 5.0f; if (g < gap) { gap = g; vl = 0; } }
    carAccel(c, v0, gap, vl, dt, amax, bcomf);
    // страховка от редких тупиков: долго стоим не на красный — проезжаем «сквозь»
    if (c.ghostT > 0) c.ghostT -= dt;
    bool nearBlock = gBlock.active && dist2d(c.pos, gBlock.pos) < 150;
    if ((c.mode == CMODE_DRIVE || c.mode == CMODE_LEAVE) && c.speed < 0.2f && !redStop && !nearBlock) { c.stuckD += dt; if (c.stuckD > 45) { c.ghostT = 2.5f; c.stuckD = 0; } } else c.stuckD = 0;
    // обгон/перестроение на трассе
    if (!c.turning && e.lanes > 1 && (c.mode == CMODE_DRIVE || c.mode == CMODE_FLEE)) {
        if (gap < 35 && vl < v0 * 0.85f && c.lane == 1 && grng.chance(dt * (c.mode == CMODE_FLEE ? 4.0f : 0.8f))) tryLaneChange(c, 0);
        else if (c.lane == 0 && gap > 60 && c.mode == CMODE_DRIVE && grng.chance(dt * 0.25f)) tryLaneChange(c, 1);
        else if (c.mode == CMODE_FLEE && gap < 25 && c.lane == 0 && grng.chance(dt * 3)) tryLaneChange(c, 1);
    }
    // беглец толкается
    if (c.mode == CMODE_FLEE && gap < 4 && c.speed < 6) { c.stuck += dt; if (c.stuck > 2.5f) { c.lat += 0; c.pressure += dt * 0.6f; } } else c.stuck = 0;
    // боковое смещение
    c.lat = approach(c.lat, c.turning ? 0 : c.latTarget, (c.mode == CMODE_FLEE ? 3.2f : 1.8f) * dt);
    // движение по пути
    float ds = c.speed * dt;
    if (!c.turning) {
        float prevEnd = e.slen - c.s;
        c.s += ds;
        int node = edgeEndNode(e, c.dir);
        if (NODES[node].light && prevEnd > 4.8f && e.slen - c.s <= 4.8f) {
            int st = lightState(node, lightAxisOf(e));
            if (st == 2 && c.mode == CMODE_DRIVE && dist2d(c.pos, PL.pos) < 110 && c.viol == 0) { c.viol = 3; c.violLimit = e.limitKmh; }
        }
        if (c.s >= e.slen) {
            if (c.mode == CMODE_PULLOVER || c.mode == CMODE_STOPPED) { /* продолжаем — найдём обочину дальше */ }
            beginTurn(c);
            c.tt = (c.s - e.slen) / c.tlen;
        }
    } else {
        c.tt += ds / c.tlen;
        if (c.tt >= 1) {
            float rem = (c.tt - 1) * c.tlen;
            c.edge = c.nEdge; c.dir = c.nDir; c.lane = c.nLane; c.s = rem; c.turning = false; c.lat = 0;
            c.planEdge = -1; planNext(c);
            if (c.mode == CMODE_DRIVE || c.mode == CMODE_FLEE) c.blink = 0;
        }
    }
    carComputePose(c);
    // прицеп
    if (c.trailer) {
        const CarModel& T = CM[MD_TRACTOR]; const CarModel& TR = CM[MD_TRAILER];
        V3 hitch = c.pos + fwdOf(c.yaw) * T.hitchZ;
        V3 axle = c.tpos + fwdOf(c.tyaw) * (-5.0f);
        V3 d = hitch - axle; c.tyaw = atan2f(d.x, d.z);
        c.tpos = hitch - fwdOf(c.tyaw) * TR.kingpinZ;
    }
    // кузов
    float al = (c.speed - c.lastSpeed) / std::max(dt, 1e-3f); c.lastSpeed = c.speed;
    float yr = wrapAngle(c.yaw - c.lastYaw) / std::max(dt, 1e-3f); c.lastYaw = c.yaw;
    float pt = clampf(-al * 0.007f, -0.04f, 0.05f), rt = clampf(yr * c.speed * 0.01f, -0.06f, 0.06f);
    c.pv += ((pt - c.pitch) * 60 - c.pv * 8) * dt; c.pitch += c.pv * dt;
    c.rv += ((rt - c.roll) * 50 - c.rv * 7) * dt; c.roll += c.rv * dt;
    c.steerVis = approach(c.steerVis, clampf(yr * 0.35f, -0.5f, 0.5f), 2 * dt);
    float wr = CM[c.model].wheels.empty() ? 0.3f : CM[c.model].wheels[0].r;
    c.spin += c.speed * dt / wr;
    if (c.flashT > 0) c.flashT -= dt;
}

// столкновения игрока с машинами
static void collidePlayerCars(float dt) {
    (void)dt;
    V3 pc[5]; int pn = carCircles(PL.pos, PL.yaw, PL.halfW, PL.halfL, pc);
    float pmass = PATROL[SV.car].mass;
    for (size_t i = 0; i < CARS.size(); i++) {
        AICar& c = CARS[i]; if (!c.alive || c.mode == CMODE_GONE) continue;
        if (dist2d(c.pos, PL.pos) > 30 && !(c.trailer && dist2d(c.tpos, PL.pos) < 20)) continue;
        V3 cc[12]; int cn = carCircles(c.pos, c.yaw, c.halfW, c.halfL, cc); int nMain = cn;
        if (c.trailer) cn += carCircles(c.tpos, c.tyaw, CM[MD_TRAILER].halfW, CM[MD_TRAILER].halfL, cc + cn);
        float cmass = CM[c.model].mass * (c.mode == CMODE_STOPPED || c.mode == CMODE_BLOCK ? 2.5f : 1.0f);
        for (int a = 0; a < pn; a++) for (int b = 0; b < cn; b++) {
            float ra = PL.halfW * 0.95f, rb = (b < nMain ? c.halfW : CM[MD_TRAILER].halfW) * 0.95f;
            V3 d = pc[a] - cc[b]; d.y = 0; float l = len(d);
            if (l < ra + rb && l > 1e-4f) {
                V3 n = d * (1 / l); float pen = ra + rb - l;
                float wp = cmass / (pmass + cmass), wc = pmass / (pmass + cmass);
                PL.pos += n * (pen * wp);
                for (int k = 0; k < pn; k++) pc[k] += n * (pen * wp);
                if (c.mode != CMODE_BLOCK && !c.trailer) { if (!c.turning) c.lat += dot(n * (-pen * wc), rightOf(c.yaw)); }
                V3 pv = fwdOf(PL.yaw) * PL.speed, cv = fwdOf(c.yaw) * c.speed;
                float rel = dot(pv - cv, -n);
                if (rel > 0.3f) {
                    PL.speed *= 1 - 0.5f * wp;
                    c.speed = std::max(0.f, c.speed - rel * wc * 0.6f);
                    if (rel > 3 && PL.hitCooldown <= 0) {
                        sfxCrash(rel / 14); PL.hitCooldown = 0.6f; gCrashFlag = rel; gLastHitCar = (int)i;
                        if (c.mode == CMODE_FLEE) c.pressure += 1.2f + rel * 0.08f * pmass;
                        for (int k = 0; k < 6; k++) emitPart((pc[a] + cc[b]) * 0.5f + V3(0, 0.6f, 0), V3(grng.range(-2, 2), grng.range(0.5f, 2), grng.range(-2, 2)), 0.3f, 0.06f, Col(1, 0.8f, 0.3f), 1, 0, true);
                    }
                }
            }
        }
    }
}

static void updateTraffic(float dt, bool spawn) {
    V3 center = PL.pos;
    int target = districtAt(center) == 0 ? 30 : 22;
    int alive = 0;
    for (size_t i = 0; i < CARS.size(); i++) {
        AICar& c = CARS[i]; if (!c.alive) continue;
        if (c.mode == CMODE_GONE) { c.alive = false; continue; }
        float d = dist2d(c.pos, center);
        if (c.vanish && (d > 90 || (c.modeT > 25 && R3.toView(c.pos).z < 0))) { c.alive = false; continue; }
        bool special = c.vanish || c.caseIdx >= 0 || (int)i == gChaseCar || c.mode == CMODE_HELPER || c.mode == CMODE_BLOCK || c.wanted;
        if (d > 470 && !special) { c.alive = false; continue; }
        if (d > 900) { c.alive = false; continue; }
        alive++;
        updateAICar(c, (int)i, dt);
        // дым из выхлопной трубы у ближних машин
        if (d < 45) {
            c.exhaustT -= dt;
            if (c.exhaustT <= 0) {
                c.exhaustT = 0.15f + grng.f() * 0.1f;
                const CarModel& M = CM[c.model];
                if (M.exhaust.y > 0) {
                    Mat3 rot = rotYPR(c.yaw, 0, 0);
                    bool heavy = c.model == MD_TRUCK || c.model == MD_TRACTOR;
                    V3 vel = heavy ? V3(0, 1.2f, 0) : fwdOf(c.yaw) * -0.8f + V3(0, 0.2f, 0);
                    emitPart(mul(rot, M.exhaust) + c.pos, vel, heavy ? 1.6f : 0.9f, heavy ? 0.2f : 0.1f, heavy ? Col(0.35f, 0.35f, 0.37f) : Col(0.65f, 0.65f, 0.67f), heavy ? 0.35f : 0.18f, 0.6f);
                }
            }
        }
    }
    if (spawn) {
        int tries = 0;
        while (alive < target && tries < 2) { if (spawnTrafficCar(center, 110, 380, true) >= 0) alive++; tries++; }
    }
}

static void drawAICar(const AICar& c, float time, bool highlight) {
    DrawCar d;
    d.pos = c.pos; d.yaw = c.yaw; d.pitch = c.pitch; d.roll = c.roll; d.paint = c.paint; d.paint2 = c.paint2;
    d.plate = c.plate; d.taxi = c.taxi; d.police = c.police || c.mode == CMODE_HELPER || c.mode == CMODE_BLOCK;
    d.flash = d.police; d.headOn = L.night > 0.35f || L.rain > 0.5f; d.brake = c.brake; d.blink = c.blink;
    d.spin = c.spin; d.steer = c.steerVis; d.time = time + (c.id % 7) * 0.13f; d.highlight = highlight;
    // тень
    float dist = dist2d(c.pos, R3.pos);
    if (dist < 150) {
        const CarModel& M = CM[c.model];
        V3 f = fwdOf(c.yaw), r = rightOf(c.yaw);
        R3.groundSprite(c.pos + V3(0, 0.08f, 0) + f * ((M.maxZ + M.minZ) * 0.5f), r * (M.halfW * 1.35f), f * (M.halfL * 1.15f), RGBA(0, 0, 0, (int)(150 * (1 - L.night * 0.5f))), 0, 0);
        if (c.trailer) R3.groundSprite(c.tpos + V3(0, 0.08f, 0), rightOf(c.tyaw) * 1.6f, fwdOf(c.tyaw) * 7.4f, RGBA(0, 0, 0, (int)(150 * (1 - L.night * 0.5f))), 0, 0);
    }
    drawCarModel(c.model, d);
    if (c.trailer) {
        DrawCar t = d; t.pos = c.tpos; t.yaw = c.tyaw; t.pitch = 0; t.roll = c.roll * 0.5f; t.plate = c.plate; t.highlight = false;
        drawCarModel(MD_TRAILER, t);
    }
    if (c.flashT > 0) R3.billboard(c.pos + V3(0, 2.5f, 0), 3, 3, RGBA(255, 255, 255, 255), 0, 3, -2);
}

static void drawPlayerCar(float time, bool cockpit) {
    DrawCar d;
    d.pos = PL.pos; d.yaw = PL.yaw; d.pitch = PL.pitch; d.roll = PL.roll;
    d.paint = Col(0.95f, 0.95f, 0.96f); d.police = true; d.flash = PL.flash; d.headOn = L.night > 0.3f || L.rain > 0.4f;
    d.brake = PL.brakeLight; d.reverse = PL.reverse; d.spin = PL.spin; d.steer = PL.steer; d.time = time; d.plate = 2;
    d.hideBody = cockpit;
    const CarModel& M = CM[PATROL[SV.car].model];
    if (!cockpit) {
        V3 f = fwdOf(PL.yaw), r = rightOf(PL.yaw);
        R3.groundSprite(PL.pos + V3(0, 0.08f, 0) + f * ((M.maxZ + M.minZ) * 0.5f), r * (M.halfW * 1.35f), f * (M.halfL * 1.15f), RGBA(0, 0, 0, (int)(160 * (1 - L.night * 0.5f))), 0, 0);
    }
    drawCarModel(PATROL[SV.car].model, d);
    if (PL.flash && L.night > 0.3f) {
        bool ph = fmodf(time, 0.5f) < 0.25f;
        R3.groundSprite(PL.pos + V3(0, 0.07f, 0), V3(7, 0, 0), V3(0, 0, 7), ph ? RGBA(120, 10, 10) : RGBA(10, 30, 140), 1, 0);
    }
}

// блокпост: две машины поперёк и барьеры
static void drawRoadblock(float time) {
    if (!gBlock.active) return;
    V3 f = fwdOf(gBlock.yaw), r = rightOf(gBlock.yaw);
    for (int k = -1; k <= 1; k += 2) {
        DrawCar d; d.pos = gBlock.pos + r * (k * 3.2f + 1.5f); d.yaw = gBlock.yaw + PI / 2 * k; d.police = true; d.flash = true; d.time = time + k; d.paint = Col(0.95f, 0.95f, 0.96f); d.plate = 3;
        d.headOn = L.night > 0.3f;
        drawCarModel(MD_SEDAN, d);
    }
    for (int i = -3; i <= 3; i++) {
        V3 p = gBlock.pos + r * (i * 2.0f + 1.5f) - f * 6;
        shadedBox(p + V3(0, 0.5f, 0), V3(0.9f, 0.18f, 0.05f), gBlock.yaw, (i & 1) ? Col(0.9f, 0.1f, 0.1f) : Col(0.95f, 0.95f, 0.95f));
        shadedBox(p + V3(0, 0.25f, 0), V3(0.05f, 0.25f, 0.2f), gBlock.yaw, Col(0.3f, 0.3f, 0.3f));
    }
    (void)f;
}

// камеры скорости в мире
static void camWorldPos(const Camera_& c, V3& pos, float& yaw) {
    const Edge& e = EDGES[c.edge];
    V3 st, f, r; float y; laneFrame(e, c.dir, st, f, r, y);
    pos = st + f * c.s + r * (e.halfW + (e.highway ? 3.0f : e.sidewalk - 0.5f)); yaw = y;
}
static void drawSpeedCams(float time) {
    (void)time;
    for (const Camera_& c : SPEEDCAMS) {
        V3 p; float y; camWorldPos(c, p, y);
        float d = dist2d(p, R3.pos); if (d > 250) continue;
        if (!R3.sphereVisible(p + V3(0, 3, 0), 4)) continue;
        shadedBox(p + V3(0, 2.5f, 0), V3(0.08f, 2.5f, 0.08f), y, Col(0.55f, 0.55f, 0.58f));
        V3 f = fwdOf(y);
        shadedBox(p + V3(0, 5.1f, 0) - f * 0.1f, V3(0.25f, 0.2f, 0.4f), y, Col(0.9f, 0.9f, 0.9f));
        shadedBox(p + V3(0, 5.1f, 0) - f * 0.55f, V3(0.12f, 0.1f, 0.06f), y, Col(0.05f, 0.05f, 0.05f));
        // знак «фотофиксация»
        shadedBox(p + V3(0, 3.2f, 0) - f * 0.12f, V3(0.3f, 0.3f, 0.02f), y, Col(0.15f, 0.35f, 0.75f));
    }
}

// =====================================================================
//  Игровая логика и интерфейс
// =====================================================================
enum Screen { SC_MENU, SC_SHIFTSTART, SC_DRIVE, SC_DOCS, SC_TRUNK, SC_DECIDE, SC_RESULT, SC_PAUSE, SC_MAP, SC_GARAGE, SC_SUMMARY, SC_HELP, SC_EXITCONFIRM };
static int gScreen = SC_MENU, gMapReturn = SC_MENU, gHelpReturn = SC_MENU;
static float gTime = 0;          // реальное время (анимации)
static bool gQuit = false;
static bool gBackground = false;
static std::string gSavePath;

static const char* RANKS[11] = {"РЯДОВОЙ", "МЛАДШИЙ СЕРЖАНТ", "СЕРЖАНТ", "СТАРШИЙ СЕРЖАНТ", "СТАРШИНА", "ПРАПОРЩИК", "МЛАДШИЙ ЛЕЙТЕНАНТ", "ЛЕЙТЕНАНТ", "СТАРШИЙ ЛЕЙТЕНАНТ", "КАПИТАН", "МАЙОР"};
static const int RANK_REP[11] = {0, 40, 100, 180, 280, 400, 550, 730, 950, 1220, 1550};
static int rankOf(int rep) { int r = 0; for (int i = 0; i < 11; i++) if (rep >= RANK_REP[i]) r = i; return r; }

// --- даты ---
static int daysFromCivil(int y, int m, int d) {
    y -= m <= 2; int era = (y >= 0 ? y : y - 399) / 400; int yoe = y - era * 400;
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1; int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}
static void civilFromDays(int z, int& y, int& m, int& d) {
    z += 719468; int era = (z >= 0 ? z : z - 146096) / 146097; int doe = z - era * 146097;
    int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; y = yoe + era * 400;
    int doy = doe - (365 * yoe + yoe / 4 - yoe / 100); int mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1; m = mp + (mp < 10 ? 3 : -9); y += (m <= 2);
}
static std::string dateStr(int days) { int y, m, d; civilFromDays(days, y, m, d); return fmt("%02d.%02d.%04d", d, m, y); }
static int TODAY() { return daysFromCivil(2026, 6, 1) + SV.day - 1; }

// --- имена ---
static const char* SUR[] = {"ИВАНОВ", "СМИРНОВ", "КУЗНЕЦОВ", "ПОПОВ", "ВАСИЛЬЕВ", "ПЕТРОВ", "СОКОЛОВ", "МИХАЙЛОВ", "НОВИКОВ", "ФЁДОРОВ", "МОРОЗОВ", "ВОЛКОВ", "АЛЕКСЕЕВ", "ЛЕБЕДЕВ", "СЕМЁНОВ",
                            "ЕГОРОВ", "ПАВЛОВ", "КОЗЛОВ", "СТЕПАНОВ", "НИКОЛАЕВ", "ОРЛОВ", "АНДРЕЕВ", "МАКАРОВ", "НИКИТИН", "ЗАХАРОВ", "ЗАЙЦЕВ", "СОЛОВЬЁВ", "БОРИСОВ", "ЯКОВЛЕВ", "ГРИГОРЬЕВ",
                            "ТИТОВ", "КРЫЛОВ", "БЕЛОВ", "ГУСЕВ", "КОМАРОВ"};
static const int NSUR = sizeof(SUR) / sizeof(SUR[0]);
static const char* MNAME[] = {"АЛЕКСАНДР", "СЕРГЕЙ", "ДМИТРИЙ", "АНДРЕЙ", "АЛЕКСЕЙ", "МАКСИМ", "ЕВГЕНИЙ", "ИВАН", "МИХАИЛ", "АРТЁМ", "НИКОЛАЙ", "ВЛАДИМИР", "ПАВЕЛ", "ОЛЕГ", "ИГОРЬ", "РОМАН", "ВИКТОР", "ЮРИЙ"};
static const char* FNAME[] = {"ЕЛЕНА", "ОЛЬГА", "НАТАЛЬЯ", "ТАТЬЯНА", "ИРИНА", "АННА", "МАРИЯ", "СВЕТЛАНА", "ЕКАТЕРИНА", "ЮЛИЯ", "МАРИНА", "АНАСТАСИЯ"};
static const char* PATR_M[] = {"АЛЕКСАНДРОВИЧ", "СЕРГЕЕВИЧ", "ДМИТРИЕВИЧ", "АНДРЕЕВИЧ", "АЛЕКСЕЕВИЧ", "ИВАНОВИЧ", "МИХАЙЛОВИЧ", "НИКОЛАЕВИЧ", "ВЛАДИМИРОВИЧ", "ПАВЛОВИЧ", "ОЛЕГОВИЧ", "ИГОРЕВИЧ", "ВИКТОРОВИЧ", "ЮРЬЕВИЧ"};
static const char* PATR_F[] = {"АЛЕКСАНДРОВНА", "СЕРГЕЕВНА", "ДМИТРИЕВНА", "АНДРЕЕВНА", "АЛЕКСЕЕВНА", "ИВАНОВНА", "МИХАЙЛОВНА", "НИКОЛАЕВНА", "ВЛАДИМИРОВНА", "ПАВЛОВНА", "ОЛЕГОВНА", "ИГОРЕВНА", "ВИКТОРОВНА", "ЮРЬЕВНА"};
static std::string surname(int i, bool female) { std::string s = SUR[i % NSUR]; if (female) s += "А"; return s; }
static std::string firstLetter(const std::string& s) { std::vector<uint32_t> c = utf8Decode(s.c_str()); if (c.empty()) return ""; char b[5] = {0}; uint32_t cp = c[0]; if (cp < 0x80) b[0] = (char)cp; else { b[0] = (char)(0xC0 | (cp >> 6)); b[1] = (char)(0x80 | (cp & 0x3F)); } return b; }

struct NamedCol { const char* n; Col c; };
static const NamedCol COLNAMES[] = {{"БЕЛЫЙ", hexc(0xf0f0f0)}, {"ЧЁРНЫЙ", hexc(0x151515)}, {"СЕРЕБРИСТЫЙ", hexc(0xa8aab0)}, {"СЕРЫЙ", hexc(0x6a6d72)}, {"СИНИЙ", hexc(0x1d2f5a)},
                                    {"ВИШНЁВЫЙ", hexc(0x7a1418)}, {"ЗЕЛЁНЫЙ", hexc(0x2a4a2e)}, {"БЕЖЕВЫЙ", hexc(0xc9b48a)}, {"ЖЁЛТЫЙ", hexc(0xf2d21a)}, {"КРАСНЫЙ", hexc(0xb02020)},
                                    {"КОРИЧНЕВЫЙ", hexc(0x5a2d1a)}, {"ГОЛУБОЙ", hexc(0x274a78)}};
static const char* colorName(const Col& c) {
    float best = 1e9f; const char* n = "СЕРЫЙ";
    for (const NamedCol& k : COLNAMES) { float d = (c.r - k.c.r) * (c.r - k.c.r) + (c.g - k.c.g) * (c.g - k.c.g) + (c.b - k.c.b) * (c.b - k.c.b); if (d < best) { best = d; n = k.n; } }
    return n;
}

// ---------------------------------------------------------------------
//  Дело (водитель, документы, багажник)
// ---------------------------------------------------------------------
enum Art { ART_SPEED, ART_RED, ART_OSAGO, ART_BELT, ART_N };
static const char* ART_NAMES[ART_N] = {"СТ. 12.9 ПРЕВЫШЕНИЕ СКОРОСТИ", "СТ. 12.12 ПРОЕЗД НА КРАСНЫЙ", "СТ. 12.37 НАРУШЕНИЕ ОСАГО", "СТ. 12.6 НЕ ПРИСТЁГНУТ РЕМЕНЬ"};
static const char* ART_SHORT[ART_N] = {"12.9", "12.12", "12.37", "12.6"};
enum Verdict { V_RELEASE, V_WARN, V_FINE, V_EVAC, V_DETAIN };
static const char* VERDICT_NAMES[5] = {"ОТПУСТИТЬ", "ПРЕДУПРЕЖДЕНИЕ", "ШТРАФ", "ЭВАКУАЦИЯ", "ЗАДЕРЖАНИЕ"};
static const char* ITEM_NAMES[] = {"СУМКА", "КОРОБКА", "КАНИСТРА", "ЧЕМОДАН", "ПАКЕТЫ", "ЯЩИК", "ЗАПАСКА", "МЕШОК"};
static const char* ITEM_LEGAL[][3] = {{"ЛИЧНЫЕ ВЕЩИ", "СПОРТИВНАЯ ФОРМА", "НОУТБУК И ЗАРЯДКА"}, {"ПОСУДА", "КНИГИ", "ЧАЙНИК С ЧЕКОМ"}, {"БЕНЗИН", "ВОДА", "ОМЫВАЙКА"},
                                     {"ОДЕЖДА", "ВЕЩИ ДЛЯ ОТПУСКА", "ДОКУМЕНТЫ ПО РАБОТЕ"}, {"ПРОДУКТЫ", "ПРОДУКТЫ", "ПОДАРКИ"}, {"ИНСТРУМЕНТЫ", "ДОМКРАТ И КЛЮЧИ", "ЗАПЧАСТИ"},
                                     {"ЗАПАСНОЕ КОЛЕСО", "ЗАПАСНОЕ КОЛЕСО", "ЗАПАСНОЕ КОЛЕСО"}, {"КАРТОШКА С ДАЧИ", "УДОБРЕНИЯ", "ЦЕМЕНТ"}};
struct TrunkItem { int kind; std::string content; bool illegal; bool inspected; V3 lp; V3 hs; Col col; };

struct Case {
    uint32_t carId = 0; int car = -1;
    bool female = false; std::string sur, name, patr; int bdate = 0; uint32_t face = 0; int age = 30;
    std::string lSur, lName, lPatr; int lBdate = 0; uint32_t lFace = 0; std::string lNum; int lIssue = 0, lValid = 0; std::string lCats; bool lForged = false;
    std::string realPlate; int realRegion = 66;
    std::string sPlate; int sRegion = 66; std::string sModel, sColor, oSur, oName, oPatr, sNum;
    bool osPresent = true; std::string osNum; int osValid = 0; std::string osPlate; int osRegion = 66; bool osUnlimited = true; bool osNotListed = false; std::string osDrivers;
    bool deprived = false; int deprivedUntil = 0; bool wanted = false; bool belt = true; bool plateMismatch = false; bool otherPerson = false;
    std::vector<TrunkItem> items; bool contraband = false; int contraKind = 0;
    int reason = 0; float violSpeed = 0, violLimit = 0; bool fled = false;
    bool baseChecked = false; float baseT = 0; bool trunkOpened = false;
    std::string speech; bool nervous = false; bool decided = false;
    // требуемое решение
    int needLevel = 0; bool needArt[ART_N] = {false, false, false, false};
    std::vector<std::string> facts;
};
static std::vector<Case> CASES;

static std::string licNumber() { return fmt("%02d %02d %06d", 10 + grng.i(89), 10 + grng.i(89), grng.i(999999)); }

static void computeTruth(Case& c) {
    c.facts.clear();
    for (int i = 0; i < ART_N; i++) c.needArt[i] = false;
    int lvl = 0;
    int today = TODAY();
    bool det = false, evac = false;
    if (c.fled) { det = true; c.facts.push_back("ПЫТАЛСЯ СКРЫТЬСЯ ОТ ПОЛИЦИИ"); }
    if (c.lForged) { det = true; c.facts.push_back("ПОДДЕЛЬНОЕ УДОСТОВЕРЕНИЕ"); }
    if (c.plateMismatch) { det = true; c.facts.push_back("НОМЕРА НЕ СОВПАДАЮТ СО СТС"); }
    if (c.wanted) { det = true; c.facts.push_back("АВТОМОБИЛЬ В УГОНЕ"); }
    if (c.contraband) { det = true; c.facts.push_back(c.contraKind == 0 ? "В БАГАЖНИКЕ КОНТРАБАНДА" : "В БАГАЖНИКЕ КРАДЕНОЕ"); }
    if (!c.lForged) {
        if (c.otherPerson) { evac = true; c.facts.push_back("ЧУЖОЕ УДОСТОВЕРЕНИЕ"); }
        else if (c.lValid < today) { evac = true; c.facts.push_back("УДОСТОВЕРЕНИЕ ПРОСРОЧЕНО"); }
        if (c.deprived) { evac = true; c.facts.push_back("ВОДИТЕЛЬ ЛИШЁН ПРАВ"); }
    }
    float over = c.violSpeed - c.violLimit;
    if (c.reason == 2 || (c.reason == 1 && over > 20)) { c.needArt[ART_SPEED] = true; c.facts.push_back(fmt("ПРЕВЫШЕНИЕ НА %d КМ/Ч", (int)over)); }
    if (c.reason == 3) { c.needArt[ART_RED] = true; c.facts.push_back("ПРОЕЗД НА КРАСНЫЙ"); }
    if (!c.osPresent) { c.needArt[ART_OSAGO] = true; c.facts.push_back("НЕТ ПОЛИСА ОСАГО"); }
    else if (c.osValid < today) { c.needArt[ART_OSAGO] = true; c.facts.push_back("ОСАГО ПРОСРОЧЕН"); }
    else if (c.osPlate != c.realPlate || c.osRegion != c.realRegion) { c.needArt[ART_OSAGO] = true; c.facts.push_back("ОСАГО НА ДРУГОЙ АВТОМОБИЛЬ"); }
    else if (c.osNotListed) { c.needArt[ART_OSAGO] = true; c.facts.push_back("ВОДИТЕЛЬ НЕ ВПИСАН В ОСАГО"); }
    if (!c.belt) { c.needArt[ART_BELT] = true; c.facts.push_back("НЕ ПРИСТЁГНУТ РЕМЕНЬ"); }
    bool anyFine = false; for (int i = 0; i < ART_N; i++) anyFine |= c.needArt[i];
    if (det) lvl = V_DETAIN; else if (evac) lvl = V_EVAC; else if (anyFine) lvl = V_FINE;
    else if (c.reason == 1) { lvl = V_WARN; c.facts.push_back(fmt("НЕЗНАЧИТЕЛЬНОЕ ПРЕВЫШЕНИЕ (+%d)", (int)over)); }
    else lvl = V_RELEASE;
    if (c.facts.empty()) c.facts.push_back("ВСЁ В ПОРЯДКЕ");
    c.needLevel = lvl;
}

static void genTrunk(Case& c, int model) {
    const CarModel& M = CM[model];
    int n = 1 + grng.i(4);
    bool big = M.tallRear;
    float floorY = big ? (model == MD_TRUCK || model == MD_TRAILER ? 1.25f : 0.62f) : M.trunkY - 0.42f;
    float z0 = M.trunkZ + 0.1f, z1 = M.trunkZ + (big ? 1.2f : 0.85f);
    if (model == MD_TRUCK) z1 = M.trunkZ + 2.2f;
    if (model == MD_PICKUP) { floorY = 0.95f; z1 = M.trunkZ + 1.4f; }
    static const Col icol[] = {hexc(0x2a3a6a), hexc(0xb08850), hexc(0xb02a20), hexc(0x3a3a3a), hexc(0xe0e0e0), hexc(0x405a30), hexc(0x151515), hexc(0x9a8a6a)};
    int illegalIdx = c.contraband ? grng.i(n) : -1;
    for (int i = 0; i < n; i++) {
        TrunkItem it; it.kind = grng.i(8); if (model == MD_TRUCK || model == MD_TRAILER) it.kind = 1;
        it.illegal = (i == illegalIdx);
        if (it.illegal) { if (it.kind == 2 || it.kind == 6) it.kind = 1; it.content = c.contraKind == 0 ? "КОНТРАБАНДА!" : "КРАДЕНОЕ!"; }
        else it.content = ITEM_LEGAL[it.kind][grng.i(3)];
        it.inspected = false; it.col = icol[it.kind];
        float sx = (i % 2 == 0 ? -1 : 1) * M.trunkW * 0.45f + grng.range(-0.08f, 0.08f);
        float sz = lerpf(z0 + 0.2f, z1 - 0.2f, (i / 2) * 0.9f + 0.05f);
        static const V3 sizes[] = {V3(0.22f, 0.14f, 0.15f), V3(0.24f, 0.2f, 0.2f), V3(0.11f, 0.2f, 0.16f), V3(0.28f, 0.12f, 0.2f), V3(0.18f, 0.13f, 0.14f), V3(0.22f, 0.1f, 0.13f), V3(0.3f, 0.1f, 0.3f), V3(0.24f, 0.16f, 0.18f)};
        it.hs = sizes[it.kind];
        if (model == MD_TRUCK) it.hs = V3(0.35f, 0.3f, 0.35f);
        it.lp = V3(sx, floorY + it.hs.y, sz);
        c.items.push_back(it);
    }
}

static int createCase(int carIdx, int reason) {
    AICar& car = CARS[carIdx];
    Case c; c.carId = car.id; c.car = carIdx;
    c.female = grng.chance(0.3f);
    int surI = grng.i(NSUR);
    c.sur = surname(surI, c.female);
    c.name = c.female ? FNAME[grng.i(12)] : MNAME[grng.i(18)];
    int pi = grng.i(14); c.patr = c.female ? PATR_F[pi] : PATR_M[pi];
    c.age = 19 + grng.i(50);
    int today = TODAY();
    c.bdate = today - c.age * 365 - grng.i(360);
    c.face = grng.next();
    c.realPlate = car.plateStr; c.realRegion = car.region;
    float p = grng.f();
    // проблемы
    c.lForged = grng.chance(0.05f);
    c.otherPerson = !c.lForged && grng.chance(0.05f);
    bool expired = !c.lForged && grng.chance(0.07f);
    c.deprived = !c.lForged && !c.otherPerson && grng.chance(0.04f);
    c.wanted = car.wanted || grng.chance(0.03f);
    c.plateMismatch = grng.chance(0.03f);
    c.contraband = grng.chance(car.wanted ? 0.5f : 0.09f); c.contraKind = grng.chance(0.55f) ? 0 : 1;
    c.belt = !grng.chance(0.1f);
    (void)p;
    // удостоверение
    if (c.otherPerson) {
        c.lSur = surname(grng.i(NSUR), c.female); c.lName = c.female ? FNAME[grng.i(12)] : MNAME[grng.i(18)];
        int pj = grng.i(14); c.lPatr = c.female ? PATR_F[pj] : PATR_M[pj];
        if (c.lSur == c.sur) c.lSur = surname(surI + 3, c.female);
        c.lFace = grng.next(); c.lBdate = c.bdate - 400 * (grng.i(10) - 5);
    } else { c.lSur = c.sur; c.lName = c.name; c.lPatr = c.patr; c.lFace = c.face; c.lBdate = c.bdate; }
    c.lNum = licNumber();
    if (expired) { c.lValid = today - 10 - grng.i(700); }
    else c.lValid = today + 30 + grng.i(3000);
    c.lIssue = c.lValid - 3652;
    int minIssue = c.lBdate + 18 * 365 + 30;
    if (c.lIssue < minIssue) { c.lIssue = minIssue; if (!expired) c.lValid = std::max(c.lValid, c.lIssue + 3652); }
    c.lCats = (car.model == MD_TRUCK || car.model == MD_TRACTOR) ? "B, C, CE" : (car.model == MD_GAZEL ? "B, D" : (grng.chance(0.3f) ? "A, B" : "B, B1"));
    if (c.deprived) c.deprivedUntil = today + 60 + grng.i(500);
    // СТС
    c.sModel = CM[car.model].name; c.sColor = colorName(car.paint);
    if (car.taxi) c.sColor = "ЖЁЛТЫЙ";
    c.sPlate = car.plateStr; c.sRegion = car.region;
    if (c.plateMismatch) { int r2; c.sPlate = randomPlate(grng, r2); c.sRegion = car.region; }
    bool ownerIsDriver = grng.chance(0.7f);
    if (ownerIsDriver) { c.oSur = c.sur; c.oName = c.name; c.oPatr = c.patr; }
    else { bool f2 = grng.chance(0.4f); c.oSur = surname(grng.i(NSUR), f2); c.oName = f2 ? FNAME[grng.i(12)] : MNAME[grng.i(18)]; int pk = grng.i(14); c.oPatr = f2 ? PATR_F[pk] : PATR_M[pk]; }
    if (c.otherPerson && ownerIsDriver) { c.oSur = c.lSur; c.oName = c.lName; c.oPatr = c.lPatr; }
    c.sNum = fmt("%02d %s%s %06d", car.region % 100, PLATE_LETTERS[grng.i(12)], PLATE_LETTERS[grng.i(12)], grng.i(999999));
    // ОСАГО
    c.osPresent = !grng.chance(0.06f);
    c.osNum = fmt("ХХХ %010d", grng.i(2000000000));
    c.osValid = grng.chance(0.06f) ? today - 5 - grng.i(200) : today + 10 + grng.i(360);
    c.osPlate = c.realPlate; c.osRegion = c.realRegion;
    if (grng.chance(0.03f)) { int r3; c.osPlate = randomPlate(grng, r3); }
    if (c.plateMismatch) { c.osPlate = c.sPlate; c.osRegion = c.sRegion; }
    c.osUnlimited = grng.chance(0.5f);
    c.osNotListed = !c.osUnlimited && grng.chance(0.12f);
    if (!c.osUnlimited) {
        std::string d1 = c.osNotListed ? surname(grng.i(NSUR), grng.chance(0.4f)) + " " + firstLetter(MNAME[grng.i(18)]) + "." : c.lSur + " " + firstLetter(c.lName) + "." + firstLetter(c.lPatr) + ".";
        c.osDrivers = d1;
        if (grng.chance(0.5f)) c.osDrivers += ", " + c.oSur + " " + firstLetter(c.oName) + ".";
    }
    // причина
    c.reason = reason; c.violSpeed = car.violSpeed; c.violLimit = car.violLimit;
    // речь
    static const char* greet[] = {"ЗДРАВСТВУЙТЕ, ИНСПЕКТОР.", "ДОБРЫЙ ДЕНЬ. ЧТО-ТО СЛУЧИЛОСЬ?", "ЗДРАСЬТЕ. Я ЧТО-ТО НАРУШИЛ?", "ОПЯТЬ ПРОВЕРКА? НУ ДЕРЖИТЕ."};
    c.speech = greet[grng.i(4)];
    if (c.female && c.speech == greet[2]) c.speech = "ЗДРАСЬТЕ. Я ЧТО-ТО НАРУШИЛА?";
    c.speech += std::string(" Я ") + c.sur + " " + c.name + ".";
    if (reason == 1 || reason == 2) c.speech += grng.chance(0.5f) ? (c.female ? " Я НЕ ЗАМЕТИЛА ЗНАК..." : " Я НЕ ЗАМЕТИЛ ЗНАК...") : " Я ОЧЕНЬ ТОРОПЛЮСЬ!";
    if (reason == 3) c.speech += " ТАМ БЫЛ ЖЁЛТЫЙ, ЧЕСТНО!";
    c.nervous = (c.contraband || c.wanted || c.lForged) && grng.chance(0.6f);
    if (!c.nervous && grng.chance(0.1f)) c.nervous = true;
    if (c.nervous) c.speech += " (ВОДИТЕЛЬ НЕРВНИЧАЕТ)";
    genTrunk(c, car.model == MD_TRACTOR ? MD_TRAILER : car.model);
    computeTruth(c);
    CASES.push_back(c);
    return (int)CASES.size() - 1;
}

// ---------------------------------------------------------------------
//  Смена, задания
// ---------------------------------------------------------------------
enum TaskType { T_CHECK, T_FINES, T_CATCH, T_SPEEDERS, T_DETAIN, T_CONTRA, T_NOERR, T_COUNT };
struct Task { int type; int need; int have; int reward; bool done; };
struct ShiftState {
    bool active = false; int district = 0; bool night = false; int weather = 0;   // 0 ясно, 1 пасмурно, 2 дождь
    float minutes = 0, startMin = 0, endMin = 0;
    Task tasks[3];
    int checks = 0, fines = 0, finesSum = 0, detains = 0, chases = 0, chaseWins = 0, correct = 0, wrong = 0, speeders = 0, contra = 0;
    int camIncome = 0, bonus = 0, penalties = 0, repDelta = 0, startRank = 0, startMoney = 0;
    float lastCamMin = 0, nextWanted = 0; int wantedCar = -1; std::string wantedDesc;
    bool tutorial = false; int tutStep = 0;
};
static ShiftState SHF;
static int gPlannedWeather = 0; static bool gPlannedNight = false; static int gPlannedDistrict = 0;

static const char* taskText(const Task& t) {
    static std::string s;
    switch (t.type) {
        case T_CHECK: s = fmt("ПРОВЕРЬ ДОКУМЕНТЫ: %d/%d", std::min(t.have, t.need), t.need); break;
        case T_FINES: s = fmt("ВЫПИШИ ШТРАФЫ: %d/%d", std::min(t.have, t.need), t.need); break;
        case T_CATCH: s = fmt("ПОЙМАЙ БЕГЛЕЦА: %d/%d", std::min(t.have, t.need), t.need); break;
        case T_SPEEDERS: s = fmt("ОСТАНОВИ ЛИХАЧЕЙ: %d/%d", std::min(t.have, t.need), t.need); break;
        case T_DETAIN: s = fmt("ЗАДЕРЖИ НАРУШИТЕЛЯ: %d/%d", std::min(t.have, t.need), t.need); break;
        case T_CONTRA: s = fmt("НАЙДИ КОНТРАБАНДУ: %d/%d", std::min(t.have, t.need), t.need); break;
        case T_NOERR: s = "СМЕНА БЕЗ ОШИБОК"; break;
    }
    return s.c_str();
}
static void genTasks() {
    int used[T_COUNT] = {0};
    for (int i = 0; i < 3; i++) {
        int t; do { t = grng.i(T_COUNT); } while (used[t]); used[t] = 1;
        Task& k = SHF.tasks[i]; k.type = t; k.have = 0; k.done = false;
        switch (t) {
            case T_CHECK: k.need = 4 + grng.i(3); k.reward = 1500; break;
            case T_FINES: k.need = 2 + grng.i(3); k.reward = 1800; break;
            case T_CATCH: k.need = 1; k.reward = 3000; break;
            case T_SPEEDERS: k.need = 2 + grng.i(2); k.reward = 2000; break;
            case T_DETAIN: k.need = 1; k.reward = 2500; break;
            case T_CONTRA: k.need = 1; k.reward = 3000; break;
            case T_NOERR: k.need = 1; k.reward = 2000; break;
        }
    }
}

// --- сообщения ---
struct Toast { std::string text; float t; int kind; };   // 0 инфо, 1 успех, 2 ошибка, 3 рация
static std::vector<Toast> TOASTS;
static void toast(const std::string& s, int kind = 0, float dur = 3.5f) {
    TOASTS.push_back({s, dur, kind});
    if (TOASTS.size() > 4) TOASTS.erase(TOASTS.begin());
}
static void taskProgress(int type, int add = 1) {
    if (!SHF.active) return;
    for (Task& t : SHF.tasks) if (t.type == type && !t.done) {
        t.have += add;
        if (t.have >= t.need) { t.done = true; SHF.bonus += t.reward; SV.money += t.reward; toast(fmt("ЗАДАНИЕ ВЫПОЛНЕНО! +%d ₽", t.reward), 1); sfx(SFX_SUCCESS); }
    }
}
static void addRep(int d) { SV.rep = std::max(0, SV.rep + d); SHF.repDelta += d; }

// ---------------------------------------------------------------------
//  Время суток и погода
// ---------------------------------------------------------------------
static float gHour = 12;
static void updateLighting(float hour, int weather) {
    float a = PI * (hour - 5.5f) / 15.0f;
    float elev = sinf(a);
    V3 sun = norm(V3(cosf(a), std::max(elev, 0.05f) * 1.2f, 0.35f));
    float day = clampf(elev * 3.0f + 0.2f, 0, 1);
    if (hour < 5 || hour > 21) day = 0;
    float sunset = clampf(1 - fabsf(elev) * 4.0f, 0, 1) * (elev > -0.1f ? 1.f : 0.f);
    L.night = 1 - day;
    L.sun = sun;
    Col topDay(0.30f, 0.52f, 0.86f), horDay(0.72f, 0.82f, 0.93f);
    Col topSet(0.30f, 0.38f, 0.65f), horSet(0.98f, 0.62f, 0.38f);
    Col topNight(0.015f, 0.025f, 0.07f), horNight(0.06f, 0.08f, 0.15f);
    Col top = lerpc(topDay, topSet, sunset * 0.8f), hor = lerpc(horDay, horSet, sunset * 0.85f);
    top = lerpc(topNight, top, day); hor = lerpc(horNight, hor, day);
    Col sunC = lerpc(Col(1, 0.96f, 0.88f), Col(1, 0.66f, 0.42f), sunset);
    Col amb = lerpc(Col(0.2f, 0.21f, 0.3f), Col(0.42f, 0.45f, 0.52f), day);
    float fogEnd = 340;
    L.rain = 0;
    if (weather >= 1) {
        Col gray = lerpc(Col(0.05f, 0.06f, 0.08f), Col(0.6f, 0.63f, 0.67f), day);
        float k = weather == 2 ? 0.85f : 0.6f;
        top = lerpc(top, gray * 0.85f, k); hor = lerpc(hor, gray, k);
        sunC = sunC * (weather == 2 ? 0.3f : 0.5f); amb = amb * 1.1f;
        fogEnd = weather == 2 ? 190 : 270;
        if (weather == 2) L.rain = 1;
    }
    if (L.night > 0.5f) fogEnd = std::min(fogEnd, 230.f);
    L.skyTop = top; L.skyHor = hor; L.fog = hor; L.sunCol = sunC; L.amb = amb;
    L.fogEnd = fogEnd; L.fogStart = fogEnd * 0.22f;
    R3.farZ = fogEnd + 30;
}

static void drawSky() {
    // горизонт: проекция удалённой точки по направлению взгляда
    float pitch = R3.pitch;
    float hy = R3.cy + tanf(pitch) * R3.focal;
    float h = clampf(hy, -SH, SH * 2);
    if (h > 0) UI.rectG(0, 0, (float)SW, std::min(h, (float)SH), toSDL(L.skyTop), toSDL(lerpc(L.skyTop, L.skyHor, 1.0f)));
    if (h < SH) UI.rect(0, std::max(h, 0.f), (float)SW, SH - std::max(h, 0.f), toSDL(L.fog));
    // солнце / луна / звёзды
    float sx, sy, sz;
    if (L.night < 0.8f) {
        if (R3.project(R3.pos + L.sun * 1000, sx, sy, sz)) {
            UI.glow(sx, sy, SH * 0.25f, toSDL(L.sunCol * (0.35f * (1 - L.night))));
            UI.circle(sx, sy, SH * 0.025f, toSDL(lerpc(L.sunCol, Col(1, 1, 1), 0.5f), (1 - L.night) * (L.rain > 0 ? 0.3f : 1)));
        }
    } else {
        V3 moon = norm(V3(-0.4f, 0.55f, 0.6f));
        if (R3.project(R3.pos + moon * 1000, sx, sy, sz)) { UI.glow(sx, sy, SH * 0.1f, RGBA(60, 70, 100)); UI.circle(sx, sy, SH * 0.02f, RGBA(230, 230, 210)); }
        if (L.rain == 0) for (int i = 0; i < 70; i++) {
            uint32_t hh = hash32(i * 977 + 13);
            float az = (hh % 6283) / 1000.0f, el = 0.08f + ((hh >> 13) % 1000) / 1000.0f * 1.2f;
            V3 d(cosf(az) * cosf(el), sinf(el), sinf(az) * cosf(el));
            if (R3.project(R3.pos + d * 1000, sx, sy, sz) && sy < h) UI.rect(sx, sy, 2, 2, RGBA(220, 220, 240, (int)(L.night * (120 + (hh % 120)))));
        }
    }
}

// --- дождь (экранный) ---
struct Drop { float x, y, v, l; };
static std::vector<Drop> DROPS;
static void drawRain(float dt, bool cockpit) {
    if (L.rain <= 0) return;
    if (DROPS.empty()) { DROPS.resize(160); for (Drop& d : DROPS) { d.x = grng.f() * SW; d.y = grng.f() * SH; d.v = SH * grng.range(1.6f, 2.4f); d.l = SH * grng.range(0.03f, 0.06f); } }
    float wind = -0.15f - fabsf(PL.speed) * 0.01f;
    for (Drop& d : DROPS) {
        d.y += d.v * dt; d.x += d.v * wind * dt;
        if (d.y > SH) { d.y = -d.l; d.x = grng.f() * SW * 1.2f; }
        UI.line(d.x, d.y, d.x + d.l * wind, d.y + d.l, std::max(1.0f, SH / 500.0f), RGBA(200, 210, 230, 70));
    }
    (void)cockpit;
}

// ---------------------------------------------------------------------
//  Ввод: касания
// ---------------------------------------------------------------------
struct Tap { float x, y; bool used; };
struct Finger { SDL_FingerID id; float x, y; };
static std::vector<Tap> TAPS;
static std::vector<Finger> FINGERS;
static bool gKeys[SDL_NUM_SCANCODES];
static bool gBackPressed = false;
static float U() { return SH / 360.0f; }

static bool tapIn(float x, float y, float w, float h) {
    for (Tap& t : TAPS) if (!t.used && t.x >= x && t.x <= x + w && t.y >= y && t.y <= y + h) { t.used = true; return true; }
    return false;
}
static bool heldIn(float x, float y, float w, float h) {
    for (const Finger& f : FINGERS) if (f.x >= x && f.x <= x + w && f.y >= y && f.y <= y + h) return true;
    return false;
}
static const SDL_Color C_PANEL = RGBA(18, 26, 40, 225), C_BLUE = RGBA(30, 95, 200), C_GREEN = RGBA(40, 150, 70), C_RED = RGBA(190, 40, 40), C_GRAY = RGBA(70, 78, 90),
                       C_WHITE = RGBA(255, 255, 255), C_YELLOW = RGBA(255, 210, 60), C_DARK = RGBA(10, 14, 22, 235), C_LIGHT = RGBA(200, 210, 225);
static bool button(float x, float y, float w, float h, const char* label, SDL_Color bg, float ts = 0, bool enabled = true) {
    if (ts <= 0) ts = U() * 1.6f;
    bool held = enabled && heldIn(x, y, w, h);
    SDL_Color c = enabled ? bg : RGBA(60, 64, 70, 200);
    if (held) c = RGBA(std::min(255, c.r + 40), std::min(255, c.g + 40), std::min(255, c.b + 40), c.a);
    UI.roundRect(x, y + U() * 1.2f, w, h, h * 0.18f, RGBA(0, 0, 0, 90));
    UI.roundRect(x, y, w, h, h * 0.18f, c);
    UI.roundRect(x + 2, y + 2, w - 4, h * 0.45f, h * 0.15f, RGBA(255, 255, 255, 22));
    float th = 7 * ts;
    // уменьшить шрифт, если не помещается
    float tw = UI.textW(label, ts);
    if (tw > w - 8) { ts *= (w - 8) / tw; th = 7 * ts; }
    UI.text(x + w * 0.5f, y + (h - th) * 0.5f, label, ts, enabled ? C_WHITE : RGBA(150, 150, 150), 1, true);
    if (enabled && tapIn(x, y, w, h)) { sfx(SFX_CLICK); return true; }
    return false;
}
static void panel(float x, float y, float w, float h, SDL_Color c = C_PANEL) { UI.roundRect(x, y, w, h, U() * 5, c); }

// ---------------------------------------------------------------------
//  Лица (фото и водитель)
// ---------------------------------------------------------------------
static void drawFace(float x, float y, float w, float h, uint32_t seed, bool female, int age, bool portrait, bool beltOn, bool nervous) {
    uint32_t s = hash32(seed);
    auto bits = [&](int sh, int n) { return (int)((s >> sh) % (uint32_t)n); };
    static const Col skins[] = {hexc(0xf3cfb3), hexc(0xe8b896), hexc(0xd9a27e), hexc(0xc58b65), hexc(0xf6d8c4)};
    static const Col hairs[] = {hexc(0x1e1610), hexc(0x3b2618), hexc(0x6b4a2a), hexc(0xa07a48), hexc(0xd8c08a), hexc(0x8a3a1a), hexc(0x777777)};
    Col skin = skins[bits(0, 5)];
    Col hair = hairs[bits(3, 6)]; if (age > 55) hair = lerpc(hair, Col(0.8f, 0.8f, 0.8f), 0.7f);
    Col shirtC[] = {hexc(0x2a3f6a), hexc(0x6a2a2a), hexc(0x3a5a3a), hexc(0x303030), hexc(0x9a9aa0), hexc(0x6a5a3a)};
    Col shirt = shirtC[bits(6, 6)];
    // фон
    if (portrait) UI.rectG(x, y, w, h, RGBA(40, 44, 52), RGBA(20, 22, 28));
    else UI.rectG(x, y, w, h, RGBA(200, 215, 230), RGBA(160, 178, 200));
    float cx = x + w * 0.5f, cy = y + h * 0.44f;
    float fw = w * (0.2f + bits(9, 5) * 0.012f), fh = h * 0.27f;
    // волосы сзади (длинные)
    int style = female ? 4 + bits(12, 3) : bits(12, 4);
    if (style == 4) UI.ellipse(cx, cy + fh * 0.3f, fw * 1.35f, fh * 1.35f, toSDL(hair));
    if (style == 5) UI.ellipse(cx, cy - fh * 0.05f, fw * 1.28f, fh * 1.15f, toSDL(hair));
    // плечи
    {
        SDL_Color sc = toSDL(shirt); int seg = 20; float rx = w * 0.48f, ry = h * 0.27f;
        for (int i = 0; i < seg; i++) { float a0 = PI + i * PI / seg, a1 = PI + (i + 1) * PI / seg; UI.tri(cx, y + h, cx + cosf(a0) * rx, y + h + sinf(a0) * ry, cx + cosf(a1) * rx, y + h + sinf(a1) * ry, sc); }
    }
    if (bits(15, 2)) UI.tri(cx - w * 0.08f, y + h * 0.75f, cx + w * 0.08f, y + h * 0.75f, cx, y + h * 0.86f, toSDL(Col(0.92f, 0.92f, 0.92f)));
    UI.rect(cx - fw * 0.38f, cy + fh * 0.6f, fw * 0.76f, h * 0.16f, toSDL(skin * 0.9f));
    if (portrait && beltOn) UI.line(x + w * 0.74f, y + h * 0.76f, x + w * 0.36f, y + h * 0.985f, w * 0.06f, RGBA(25, 25, 28));
    // уши
    UI.ellipse(cx - fw, cy + fh * 0.05f, fw * 0.18f, fh * 0.2f, toSDL(skin * 0.92f));
    UI.ellipse(cx + fw, cy + fh * 0.05f, fw * 0.18f, fh * 0.2f, toSDL(skin * 0.92f));
    // голова
    UI.ellipse(cx, cy, fw, fh, toSDL(skin), 28);
    if (nervous && portrait) { UI.circle(cx + fw * 0.6f, cy - fh * 0.5f, fw * 0.08f, RGBA(180, 220, 255, 200)); }
    // волосы сверху
    SDL_Color hc = toSDL(hair);
    switch (style) {
        case 0: UI.ellipse(cx, cy - fh * 0.62f, fw * 1.02f, fh * 0.45f, hc); break;
        case 1: UI.ellipse(cx - fw * 0.85f, cy - fh * 0.15f, fw * 0.2f, fh * 0.35f, hc); UI.ellipse(cx + fw * 0.85f, cy - fh * 0.15f, fw * 0.2f, fh * 0.35f, hc); break;
        case 2: UI.ellipse(cx, cy - fh * 0.75f, fw * 0.95f, fh * 0.3f, hc); break;
        case 3: UI.ellipse(cx - fw * 0.15f, cy - fh * 0.62f, fw * 1.05f, fh * 0.48f, hc); UI.tri(cx - fw, cy - fh * 0.5f, cx + fw * 0.4f, cy - fh * 0.7f, cx - fw * 0.9f, cy - fh * 0.05f, hc); break;
        case 4: UI.ellipse(cx, cy - fh * 0.6f, fw * 1.1f, fh * 0.5f, hc); UI.rect(cx - fw * 1.1f, cy - fh * 0.5f, fw * 0.3f, fh * 1.4f, hc); UI.rect(cx + fw * 0.8f, cy - fh * 0.5f, fw * 0.3f, fh * 1.4f, hc); break;
        case 5: UI.ellipse(cx, cy - fh * 0.6f, fw * 1.12f, fh * 0.5f, hc); break;
        case 6: UI.ellipse(cx, cy - fh * 0.62f, fw * 1.05f, fh * 0.45f, hc); UI.circle(cx, cy - fh * 1.08f, fw * 0.38f, hc); break;
    }
    // глаза и брови
    float ey = cy - fh * 0.08f, ex = fw * 0.42f, er = fw * (0.15f + bits(18, 3) * 0.02f);
    for (int k = -1; k <= 1; k += 2) {
        UI.ellipse(cx + k * ex, ey, er, er * 0.65f, RGBA(245, 245, 245), 12);
        UI.circle(cx + k * ex + (nervous ? fw * 0.06f : 0), ey, er * 0.48f, toSDL(bits(20, 2) ? Col(0.25f, 0.35f, 0.5f) : Col(0.3f, 0.2f, 0.12f)), 10);
        UI.circle(cx + k * ex + (nervous ? fw * 0.06f : 0), ey, er * 0.22f, RGBA(10, 10, 10), 8);
        float bt = fh * (0.05f + bits(22, 3) * 0.02f);
        UI.line(cx + k * (ex - er), ey - er * 1.2f - (k * bits(24, 2)) * 1.0f, cx + k * (ex + er * 1.1f), ey - er * 1.35f, bt, toSDL(hair * 0.8f));
    }
    // нос
    float nl = fh * (0.25f + bits(26, 3) * 0.05f);
    UI.tri(cx, ey + er * 0.4f, cx - fw * 0.12f, ey + nl, cx + fw * 0.08f, ey + nl, toSDL(skin * 0.82f));
    // рот
    float my = cy + fh * 0.5f, mw = fw * (0.28f + bits(28, 3) * 0.05f);
    UI.line(cx - mw, my, cx + mw, my + (nervous ? fh * 0.03f : 0), fh * 0.06f, female ? RGBA(180, 50, 60) : toSDL(skin * 0.6f));
    if (!female && bits(29, 3) == 0) UI.line(cx - mw * 1.1f, my - fh * 0.1f, cx + mw * 1.1f, my - fh * 0.1f, fh * 0.09f, toSDL(hair * 0.9f));
    if (!female && bits(30, 6) == 0) { UI.ellipse(cx, cy + fh * 0.7f, fw * 0.75f, fh * 0.35f, toSDL(hair * 0.9f)); UI.line(cx - mw, my, cx + mw, my, fh * 0.05f, toSDL(skin * 0.5f)); }
    if (age > 50) for (int k = 0; k < 2; k++) UI.line(cx - fw * 0.4f, cy - fh * (0.45f + k * 0.1f), cx + fw * 0.4f, cy - fh * (0.45f + k * 0.1f), 1.5f, toSDL(skin * 0.8f));
    if (bits(5, 5) == 0) {
        for (int k = -1; k <= 1; k += 2) UI.ring(cx + k * ex, ey, er * 1.25f, er * 1.5f, RGBA(30, 30, 30), 16);
        UI.line(cx - ex + er * 1.3f, ey, cx + ex - er * 1.3f, ey, 2, RGBA(30, 30, 30));
    }
}

// ---------------------------------------------------------------------
//  Камера (вид)
// ---------------------------------------------------------------------
static V3 gCamPos; static float gCamYaw = 0, gCamPitch = -0.16f;
static int gCamMode = 0;   // 0 сзади, 1 из кабины
static void updateCamera(float dt, bool snap) {
    const CarModel& M = CM[PATROL[SV.car].model];
    if (gCamMode == 1) {
        Mat3 rot = rotYPR(PL.yaw, PL.pitch, PL.roll);
        gCamPos = mul(rot, V3(-0.37f * M.halfW / 0.9f, M.roofY * 0.8f, (M.maxZ + M.minZ) * 0.5f - 0.05f)) + PL.pos;
        gCamYaw = PL.yaw; gCamPitch = -0.05f - PL.pitch;
        R3.setCamera(gCamPos, gCamYaw, gCamPitch, 72);
        return;
    }
    float back = 4.8f + M.halfL * 1.0f + fabsf(PL.speed) * 0.04f, up = 1.6f + M.height * 0.9f;
    float yawT = PL.yaw; if (PL.speed < -1) yawT = PL.yaw;
    if (snap) gCamYaw = yawT;
    gCamYaw = wrapAngle(gCamYaw + wrapAngle(yawT - gCamYaw) * clampf(dt * 5.5f, 0, 1));
    V3 target = PL.pos - fwdOf(gCamYaw) * back + V3(0, up, 0);
    if (snap) gCamPos = target; else gCamPos = lerp3(gCamPos, target, clampf(dt * 8, 0, 1));
    // не уходить под землю/в стены
    V3 p = gCamPos; collideStatic(p, 0.4f, nullptr); gCamPos = p;
    if (gCamPos.y < 0.6f) gCamPos.y = 0.6f;
    V3 look = PL.pos + V3(0, M.height * 0.65f, 0) + fwdOf(PL.yaw) * 3.0f;
    V3 d = look - gCamPos;
    float pitch = atan2f(d.y, sqrtf(d.x * d.x + d.z * d.z));
    gCamPitch = pitch;
    R3.setCamera(gCamPos, gCamYaw, gCamPitch, 64 + std::min(fabsf(PL.speed) * 0.25f, 10.f));
}

// ---------------------------------------------------------------------
//  Сцена
// ---------------------------------------------------------------------
static int gTarget = -1;      // машина-цель (подсвечена)
static int gInspect = -1;     // осматриваемая машина
static void drawWorldScene(float dt, bool drawPlayer, bool cockpit) {
    L.hlOn = (L.night > 0.4f) ? 1.0f : 0.0f;
    const CarModel& M = CM[PATROL[SV.car].model];
    L.hlPos = PL.pos + fwdOf(PL.yaw) * M.maxZ + V3(0, 0.7f, 0); L.hlDir = fwdOf(PL.yaw);
    drawSky();
    UI.flush();
    drawGround();
    drawRoads();
    drawStatic();
    drawGuardRails();
    for (const Ped& p : PEDS) if (p.alive) drawPed(p);
    for (size_t i = 0; i < CARS.size(); i++) if (CARS[i].alive) drawAICar(CARS[i], gTime, (int)i == gTarget);
    drawRoadblock(gTime);
    drawSpeedCams(gTime);
    if (drawPlayer) drawPlayerCar(gTime, cockpit);
    drawParts();
    R3.draw();
    (void)dt;
}

// ---------------------------------------------------------------------
//  Сохранение
// ---------------------------------------------------------------------
static void saveGame() {
    if (gSavePath.empty()) return;
    FILE* f = fopen(gSavePath.c_str(), "w");
    if (!f) return;
    fprintf(f, "money %d\nrep %d\nday %d\nowned %d\ncar %d\n", SV.money, SV.rep, SV.day, SV.carOwned, SV.car);
    for (int i = 0; i < U_COUNT; i++) fprintf(f, "upg%d %d\n", i, SV.upg[i]);
    fprintf(f, "complaints %d\nchecks %d\nfines %d\ndetain %d\nchases %d\ncorrect %d\nwrong %d\n", SV.complaints, SV.totalChecks, SV.totalFines, SV.totalDetain, SV.totalChases, SV.totalCorrect, SV.totalWrong);
    fprintf(f, "sound %d\ncamview %d\ndistrict %d\ntutorial %d\n", SV.sound ? 1 : 0, SV.camView, SV.lastDistrict, SV.tutorialShown ? 1 : 0);
    for (const Camera_& c : SPEEDCAMS) fprintf(f, "cam %d %d %.1f\n", c.edge, c.dir, c.s);
    fclose(f);
}
static void loadGame() {
    if (gSavePath.empty()) return;
    FILE* f = fopen(gSavePath.c_str(), "r");
    if (!f) return;
    char key[64]; SPEEDCAMS.clear();
    while (fscanf(f, "%63s", key) == 1) {
        if (!strcmp(key, "cam")) { Camera_ c; if (fscanf(f, "%d %d %f", &c.edge, &c.dir, &c.s) == 3 && c.edge >= 0 && c.edge < (int)EDGES.size()) SPEEDCAMS.push_back(c); continue; }
        int v = 0; if (fscanf(f, "%d", &v) != 1) break;
        if (!strcmp(key, "money")) SV.money = v; else if (!strcmp(key, "rep")) SV.rep = v; else if (!strcmp(key, "day")) SV.day = std::max(1, v);
        else if (!strcmp(key, "owned")) SV.carOwned = v | 1; else if (!strcmp(key, "car")) SV.car = clampf((float)v, 0, 3);
        else if (!strncmp(key, "upg", 3)) { int i = atoi(key + 3); if (i >= 0 && i < U_COUNT) SV.upg[i] = std::max(0, std::min(3, v)); }
        else if (!strcmp(key, "complaints")) SV.complaints = v; else if (!strcmp(key, "checks")) SV.totalChecks = v; else if (!strcmp(key, "fines")) SV.totalFines = v;
        else if (!strcmp(key, "detain")) SV.totalDetain = v; else if (!strcmp(key, "chases")) SV.totalChases = v; else if (!strcmp(key, "correct")) SV.totalCorrect = v;
        else if (!strcmp(key, "wrong")) SV.totalWrong = v; else if (!strcmp(key, "sound")) SV.sound = v != 0; else if (!strcmp(key, "camview")) SV.camView = v;
        else if (!strcmp(key, "district")) SV.lastDistrict = std::max(0, std::min(3, v)); else if (!strcmp(key, "tutorial")) SV.tutorialShown = v != 0;
    }
    fclose(f);
    if (!(SV.carOwned & (1 << SV.car))) SV.car = 0;
}

// ---------------------------------------------------------------------
//  Начало / конец смены
// ---------------------------------------------------------------------
static void clearWorldActors() {
    for (AICar& c : CARS) c.alive = false;
    CARS.clear(); CASES.clear(); PARTS.clear(); PEDS.clear();
    gChaseCar = -1; gBlock.active = false; gTarget = -1; gInspect = -1;
}
static void startShift() {
    clearWorldActors();
    SHF = ShiftState();
    SHF.active = true; SHF.district = gPlannedDistrict; SHF.night = gPlannedNight; SHF.weather = gPlannedWeather;
    SHF.startMin = SHF.night ? 20 * 60 : 8 * 60; SHF.minutes = SHF.startMin; SHF.endMin = SHF.startMin + 8 * 60;
    SHF.lastCamMin = SHF.minutes; SHF.nextWanted = SHF.minutes + 50 + grng.i(60);
    SHF.startRank = rankOf(SV.rep); SHF.startMoney = SV.money;
    SHF.tutorial = !SV.tutorialShown; SHF.tutStep = 0;
    genTasks();
    TOASTS.clear();
    SV.lastDistrict = SHF.district;
    const Post& P = POSTS[SHF.district];
    PL = PlayerCar(); playerSetCar();
    PL.pos = P.pos; PL.yaw = P.yaw;
    gCamMode = SV.camView;
    gWorldTime = grng.range(0, 100);
    gHour = SHF.minutes / 60.0f;
    updateLighting(fmodf(gHour, 24), SHF.weather);
    updateCamera(0, true);
    // начальный трафик
    for (int i = 0; i < 26; i++) spawnTrafficCar(PL.pos, 40, 380, false);
    platePixels(2, "А0166", 66, true); platePixels(3, "А0266", 66, true);
    gScreen = SC_DRIVE;
    toast(fmt("СМЕНА НАЧАЛАСЬ. УЧАСТОК: %s", DISTRICT_NAMES[SHF.district]), 0, 4);
    if (SHF.tutorial) toast("ПОДСКАЗКА: НАЙДИ НАРУШИТЕЛЯ РАДАРОМ, ВКЛЮЧИ МИГАЛКУ И НАЖМИ «ТРЕБОВАТЬ»", 3, 7);
    sfx(SFX_RADIO, 0.6f, 0, 0, 1.2f);
}
static void endShift() {
    SHF.active = false;
    // зарплата
    int salary = 2500 + rankOf(SV.rep) * 400;
    if (SHF.night) salary += 800;
    float worked = clampf((SHF.minutes - SHF.startMin) / (SHF.endMin - SHF.startMin), 0, 1);
    salary = (int)(salary * worked);
    for (Task& t : SHF.tasks) if (t.type == T_NOERR && !t.done && SHF.wrong == 0 && SHF.checks >= 3 && worked > 0.95f) { t.done = true; SHF.bonus += t.reward; SV.money += t.reward; }
    SV.money += salary; SHF.bonus += 0;
    SHF.camIncome += 0;
    SHF.penalties += 0;
    SHF.checks += 0;
    SHF.startMoney = SV.money - SHF.startMoney;  // теперь — итог за смену
    SHF.lastCamMin = (float)salary;             // сохраняем зарплату для экрана итогов
    SV.day++;
    sfx(SFX_SUCCESS);
    saveGame();
    gScreen = SC_SUMMARY;
}

// ---------------------------------------------------------------------
//  Логика остановки
// ---------------------------------------------------------------------
static float gRadarSpeed = -1; static int gRadarCar = -1; static float gRadarLimit = 60;
static float gDemandCooldown = 0; static float gHelpCooldown = 0, gBlockCooldown = 0; static bool gHelpUsed = false, gBlockUsed = false;
static float gChaseFarT = 0; static float gCrashRepCd = 0; static float gTutHintT = 0;
static std::string gSubtitle; static float gSubtitleT = 0;

static void subtitle(const std::string& s, float t = 2.5f) { gSubtitle = s; gSubtitleT = t; }

static void pickTarget() {
    gTarget = -1; float best = 1e9f;
    V3 f = fwdOf(PL.yaw);
    bool parked = fabsf(PL.speed) < 1.5f;
    for (size_t i = 0; i < CARS.size(); i++) {
        const AICar& c = CARS[i];
        if (!c.alive || c.mode == CMODE_HELPER || c.mode == CMODE_BLOCK || c.mode == CMODE_GONE || c.mode == CMODE_STOPPED || c.mode == CMODE_PULLOVER || c.mode == CMODE_LEAVE) continue;
        if ((int)i == gChaseCar) continue;
        if (c.checked) continue;
        V3 d = c.pos - PL.pos; float dd = len(d);
        if (dd > (parked ? 45.f : 75.f) || dd < 2) continue;
        float cs = dot(d * (1 / dd), f);
        if (!parked && cs < 0.75f) continue;
        if (parked && cs < -0.2f) continue;
        float score = dd * (c.viol > 0 ? 0.4f : 1.0f) * (c.wanted ? 0.3f : 1.0f);
        if (score < best) { best = score; gTarget = (int)i; }
    }
}

static void updateRadar() {
    gRadarSpeed = -1; gRadarCar = -1;
    float range = RADAR_RANGE[SV.upg[U_RADAR]];
    V3 f = fwdOf(PL.yaw);
    float best = 1e9f;
    for (size_t i = 0; i < CARS.size(); i++) {
        AICar& c = CARS[i]; if (!c.alive || c.mode != CMODE_DRIVE) continue;
        V3 d = c.pos - PL.pos; float dd = len(d);
        if (dd > range || dd < 3) continue;
        float cs = dot(d * (1 / dd), f); if (cs < 0.93f) continue;
        if (dd < best) { best = dd; gRadarCar = (int)i; }
    }
    if (gRadarCar >= 0) {
        AICar& c = CARS[gRadarCar];
        const Edge& e = EDGES[c.edge];
        gRadarSpeed = c.speed * 3.6f; gRadarLimit = e.limitKmh;
        float over = gRadarSpeed - e.limitKmh;
        if (over > 20 && c.viol != 3) { if (c.viol != 2) toast(fmt("РАДАР: %d КМ/Ч ПРИ ОГРАНИЧЕНИИ %d!", (int)gRadarSpeed, (int)e.limitKmh), 2, 2.5f); c.viol = 2; c.violSpeed = std::max(c.violSpeed, gRadarSpeed); c.violLimit = e.limitKmh; }
        else if (over > 3 && c.viol == 0) { c.viol = 1; c.violSpeed = gRadarSpeed; c.violLimit = e.limitKmh; }
    }
}

static bool seriousIssue(const Case& c) { return c.lForged || c.wanted || c.contraband || c.deprived || c.plateMismatch || c.otherPerson; }

static void demandStop() {
    if (gTarget < 0 || gDemandCooldown > 0) return;
    AICar& c = CARS[gTarget];
    gDemandCooldown = 2.0f;
    bool parked = fabsf(PL.speed) < 1.5f;
    if (parked) { sfx(SFX_WHISTLE, 0.7f); subtitle("ИНСПЕКТОР ПОКАЗЫВАЕТ ЖЕЗЛОМ: К ОБОЧИНЕ!"); }
    else { sfx(SFX_SPEECH, 0.9f, 115, 0, 1.6f); subtitle(fmt("«ВОДИТЕЛЬ %s %s, ПРИЖМИТЕСЬ К ОБОЧИНЕ!»", CM[c.model].name.c_str(), c.plateStr.c_str())); }
    c.demands++; c.reactT = 1.3f;
}

static void startChase(int idx) {
    AICar& c = CARS[idx];
    c.mode = CMODE_FLEE; c.modeT = 0; c.pressure = 0; c.trail.clear(); c.blink = 0; c.latTarget = 0;
    gChaseCar = idx; gChaseFarT = 0; gHelpUsed = false; gBlockUsed = false; gHelpCooldown = 0; gBlockCooldown = 0;
    SHF.chases++; SV.totalChases++;
    toast("НАРУШИТЕЛЬ ПЫТАЕТСЯ СКРЫТЬСЯ! ПОГОНЯ!", 2, 3.5f);
    sfx(SFX_RADIO, 0.6f);
    if (!PL.siren) { PL.siren = true; PL.flash = true; }
}
static void endChase(bool success) {
    if (gChaseCar < 0) return;
    AICar& c = CARS[gChaseCar];
    for (AICar& h : CARS) if (h.alive && h.mode == CMODE_HELPER) h.mode = CMODE_GONE;
    gBlock.active = false;
    if (success) {
        c.mode = CMODE_PULLOVER; c.caught = true; c.speed = std::min(c.speed, 12.f);
        if (c.caseIdx < 0) c.caseIdx = createCase(gChaseCar, 0);
        Case& cs = CASES[c.caseIdx]; cs.fled = true; computeTruth(cs);
        SHF.chaseWins++;
        toast("НАРУШИТЕЛЬ ОСТАНОВЛЕН! ПРОВЕРЬ ДОКУМЕНТЫ.", 1, 4);
        taskProgress(T_CATCH);
        sfx(SFX_SUCCESS);
    } else {
        c.alive = false;
        toast("НАРУШИТЕЛЬ СКРЫЛСЯ. ПОГОНЯ ПРОВАЛЕНА.", 2, 4);
        addRep(-5); sfx(SFX_ERROR);
    }
    gChaseCar = -1;
}

static void requestHelp() {
    if (gChaseCar < 0 || gHelpUsed) return;
    AICar& t = CARS[gChaseCar];
    if (t.trail.size() < 4) { toast("РАЦИЯ: УТОЧНИТЕ НАПРАВЛЕНИЕ...", 3); return; }
    AICar h; h.alive = true; h.id = gCarIdCounter++; h.model = MD_SEDAN; h.police = true; h.paint = Col(0.95f, 0.95f, 0.96f);
    h.mode = CMODE_HELPER; h.helperTarget = gChaseCar;
    int start = 0;
    for (int k = (int)t.trail.size() - 1; k >= 0; k--) { if (dist2d(t.trail[k], t.pos) > 35 && dist2d(t.trail[k], PL.pos) > 30) { start = k; break; } }
    h.pos = t.trail[start]; h.trailIdx = start + 1;
    V3 d = t.trail[std::min((int)t.trail.size() - 1, start + 1)] - h.pos; h.yaw = atan2f(d.x, d.z);
    h.speed = t.speed; h.halfW = CM[MD_SEDAN].halfW; h.halfL = CM[MD_SEDAN].halfL; h.plate = 3; h.plateStr = "А0266";
    CARS.push_back(h);
    gHelpUsed = true;
    toast("РАЦИЯ: ЭКИПАЖ 2-14 ПОДКЛЮЧИЛСЯ К ПРЕСЛЕДОВАНИЮ!", 3, 4);
    sfx(SFX_RADIO, 0.7f, 0, 0, 1.4f);
}
static void requestRoadblock() {
    if (gChaseCar < 0 || gBlockUsed) return;
    AICar& t = CARS[gChaseCar];
    int edge, dir; float s;
    if (!t.turning && EDGES[t.edge].slen - t.s > 200) { edge = t.edge; dir = t.dir; s = t.s + 170; }
    else {
        if (t.planEdge < 0) planNext(t);
        edge = t.turning ? t.nEdge : t.planEdge; dir = t.turning ? t.nDir : t.planDir;
        s = std::min(120.f, EDGES[edge].slen * 0.6f);
    }
    const Edge& e = EDGES[edge];
    V3 st, f, r; float y; laneFrame(e, dir, st, f, r, y);
    gBlock.active = true; gBlock.edge = edge; gBlock.dir = dir; gBlock.s = s; gBlock.yaw = y;
    gBlock.pos = st + f * s + r * (e.halfW * 0.5f);
    gBlock.pos = st + f * s + r * (e.halfW * 0.5f) - r * 1.5f;
    gBlockUsed = true;
    toast("РАЦИЯ: БЛОКПОСТ ВЫСТАВЛЕН ВПЕРЕДИ ПО ХОДУ!", 3, 4);
    sfx(SFX_RADIO, 0.7f, 0, 0, 1.4f);
}

static void spawnWanted() {
    int idx = spawnTrafficCar(PL.pos, 160, 330, false);
    if (idx < 0) return;
    AICar& c = CARS[idx];
    c.wanted = true;
    SHF.wantedCar = idx;
    SHF.wantedDesc = fmt("%s, %s, %s %d", CM[c.model].name.c_str(), colorName(c.paint), c.plateStr.c_str(), c.region);
    toast("ОРИЕНТИРОВКА: РАЗЫСКИВАЕТСЯ " + SHF.wantedDesc, 3, 7);
    sfx(SFX_RADIO, 0.7f, 0, 0, 2.0f);
}

static void reactToDemand(int idx) {
    AICar& c = CARS[idx];
    if (c.caseIdx < 0) {
        int reason = 0;
        if (c.viol == 2) reason = 2; else if (c.viol == 3) reason = 3; else if (c.viol == 1) reason = 1;
        c.caseIdx = createCase(idx, reason);
    }
    Case& cs = CASES[c.caseIdx];
    float lightsK = SV.upg[U_LIGHTS] * 0.05f;
    float pFlee = 0.04f + (seriousIssue(cs) ? 0.4f : 0) + (c.wanted ? 0.3f : 0) - lightsK + (c.demands > 1 ? 0.15f : 0);
    float pIgnore = (PL.flash ? 0.1f : 0.32f) - lightsK + (c.demands > 2 ? -0.2f : 0);
    if (c.trailer || c.model == MD_TRUCK) pFlee *= 0.2f;
    // задание «поймай беглеца» — чаще бегут
    for (Task& t : SHF.tasks) if (t.type == T_CATCH && !t.done && SHF.chases == 0) pFlee += 0.12f;
    float r = grng.f();
    if (r < pFlee) { startChase(idx); return; }
    if (r < pFlee + pIgnore) { c.ignoring = true; toast("ВОДИТЕЛЬ ДЕЛАЕТ ВИД, ЧТО НЕ ЗАМЕТИЛ...", 0, 2.5f); return; }
    c.mode = CMODE_PULLOVER; c.modeT = 0; c.ignoring = false; c.yieldT = 0;
    if (c.viol == 1 || c.viol == 2) { SHF.speeders++; taskProgress(T_SPEEDERS); }
    toast("ВОДИТЕЛЬ ОСТАНАВЛИВАЕТСЯ.", 0, 2);
}

static void finishInspection(int verdict, bool* arts, std::string& outTitle, std::vector<std::string>& outLines, int& moneyD, int& repD, bool& good) {
    AICar& car = CARS[gInspect];
    Case& c = CASES[car.caseIdx];
    computeTruth(c);
    c.decided = true; car.checked = true;
    SHF.checks++; SV.totalChecks++; taskProgress(T_CHECK);
    moneyD = 0; repD = 0; good = false;
    int need = c.needLevel;
    bool artOk = true; int artHit = 0, artNeed = 0, artExtra = 0;
    if (verdict == V_FINE) {
        for (int i = 0; i < ART_N; i++) { if (c.needArt[i]) artNeed++; if (arts[i] && c.needArt[i]) artHit++; if (arts[i] && !c.needArt[i]) artExtra++; }
        artOk = (artHit == artNeed && artExtra == 0);
    }
    outLines.clear();
    bool acceptable = (verdict == need) || (need == V_WARN && verdict == V_FINE && arts[ART_SPEED] && artExtra == 0);
    if (verdict == V_FINE && need == V_FINE && !artOk) acceptable = false;
    if (acceptable) {
        good = true;
        static const int rewards[5] = {150, 250, 450, 900, 1600};
        static const int reps[5] = {2, 3, 4, 6, 9};
        moneyD = rewards[verdict]; repD = reps[verdict];
        if (verdict == V_FINE) {
            int sum = 0;
            for (int i = 0; i < ART_N; i++) if (arts[i]) {
                int a = i == ART_SPEED ? ((c.violSpeed - c.violLimit) > 40 ? 1500 : 500) : (i == ART_RED ? 1000 : (i == ART_OSAGO ? 800 : 1000));
                sum += a;
            }
            moneyD += sum / 10; SHF.fines++; SHF.finesSum += sum; SV.totalFines++; taskProgress(T_FINES);
            outLines.push_back(fmt("ШТРАФ ВЫПИСАН НА %d ₽", sum));
        }
        if (verdict == V_DETAIN) { SHF.detains++; SV.totalDetain++; taskProgress(T_DETAIN); if (c.fled) moneyD += 1000; }
        if (c.contraband && c.trunkOpened) { bool found = false; for (auto& it : c.items) if (it.illegal && it.inspected) found = true; if (found) { SHF.contra++; taskProgress(T_CONTRA); moneyD += 1500; } }
        if (c.wanted && car.wanted) { moneyD += 2500; outLines.push_back("РАЗЫСКИВАЕМЫЙ АВТОМОБИЛЬ ЗАДЕРЖАН!"); }
        SHF.correct++; SV.totalCorrect++;
        outTitle = "ВЕРНОЕ РЕШЕНИЕ";
    } else {
        SHF.wrong++; SV.totalWrong++;
        if (verdict > need || (verdict == V_FINE && need == V_FINE && artExtra > 0)) {
            outTitle = "ПРИДИРКА! ВОДИТЕЛЬ ПОДАЛ ЖАЛОБУ";
            moneyD = -500; repD = -6; SV.complaints++;
        } else if (verdict == V_FINE && need == V_FINE) {
            outTitle = "НЕВЕРНЫЕ СТАТЬИ В ПРОТОКОЛЕ";
            moneyD = 100; repD = -1;
            SHF.fines++; SV.totalFines++; taskProgress(T_FINES);
        } else {
            outTitle = need == V_DETAIN ? "ОПАСНЫЙ НАРУШИТЕЛЬ ОТПУЩЕН!" : "НАРУШЕНИЕ ПРОПУЩЕНО";
            moneyD = need == V_DETAIN ? -800 : -300; repD = need == V_DETAIN ? -10 : -4;
        }
        SHF.penalties += -std::min(0, moneyD);
    }
    outLines.push_back(std::string("ТРЕБОВАЛОСЬ: ") + VERDICT_NAMES[need]);
    for (auto& f : c.facts) outLines.push_back("- " + f);
    if (need == V_FINE) {
        std::string a = "СТАТЬИ:"; for (int i = 0; i < ART_N; i++) if (c.needArt[i]) a += std::string(" ") + ART_SHORT[i];
        outLines.push_back(a);
    }
    SV.money = std::max(0, SV.money + moneyD); addRep(repD);
    if (good) sfx(SFX_COIN); else sfx(SFX_ERROR);
    // судьба машины
    if (verdict == V_EVAC || verdict == V_DETAIN) {
        car.mode = CMODE_STOPPED; car.vanish = true; car.caseIdx = -1; car.modeT = 0; car.viol = 0;
        outLines.push_back(verdict == V_EVAC ? "АВТОМОБИЛЬ ОТПРАВЛЕН НА ШТРАФСТОЯНКУ." : "НАРУШИТЕЛЬ ДОСТАВЛЕН В ДЕЖУРНУЮ ЧАСТЬ.");
    }
    else { car.mode = CMODE_LEAVE; car.modeT = 0; car.caseIdx = -1; car.viol = 0; }
    SHF.minutes += 3;
}

// ---------------------------------------------------------------------
//  Обновление вождения
// ---------------------------------------------------------------------
static Input gIn;
static void updateDrive(float dt) {
    SHF.minutes += dt * (40.0f / 60.0f);
    gHour = fmodf(SHF.minutes / 60.0f, 24);
    updateLighting(gHour, SHF.weather);
    gWorldTime += dt;
    updatePlayer(dt, gIn);
    collidePlayerCars(dt);
    updateTraffic(dt, true);
    updatePeds(dt, PL.pos, true);
    updateParts(dt);
    updateRadar();
    pickTarget();
    if (gDemandCooldown > 0) gDemandCooldown -= dt;
    if (gCrashRepCd > 0) gCrashRepCd -= dt;
    if (gSubtitleT > 0) gSubtitleT -= dt;
    for (size_t i = 0; i < TOASTS.size();) { TOASTS[i].t -= dt; if (TOASTS[i].t <= 0) TOASTS.erase(TOASTS.begin() + i); else i++; }
    // реакции на требование
    for (size_t i = 0; i < CARS.size(); i++) {
        AICar& c = CARS[i];
        if (!c.alive || c.reactT < 0) continue;
        c.reactT -= dt;
        if (c.reactT < 0 && c.mode == CMODE_DRIVE) reactToDemand((int)i);
    }
    // ДТП с мирными машинами
    if (gCrashFlag > 0) {
        if (gLastHitCar >= 0 && gLastHitCar != gChaseCar && gCrashFlag > 7 && gCrashRepCd <= 0 && CARS[gLastHitCar].alive && CARS[gLastHitCar].mode != CMODE_HELPER) {
            addRep(-2); toast("ДТП С УЧАСТИЕМ ИНСПЕКТОРА! -2 РЕПУТАЦИИ", 2, 3); gCrashRepCd = 5;
        }
        gCrashFlag = 0; gLastHitCar = -1;
    }
    // погоня
    if (gChaseCar >= 0) {
        AICar& c = CARS[gChaseCar];
        if (!c.alive) { gChaseCar = -1; }
        else {
            float d = dist2d(c.pos, PL.pos);
            if (d < 16) c.pressure += dt * (0.35f + PATROL[SV.car].mass * 0.12f);
            if (gBlock.active && dist2d(c.pos, gBlock.pos) < 20 && c.speed < 3) c.pressure = 99;
            if (d > 280) gChaseFarT += dt; else gChaseFarT = std::max(0.f, gChaseFarT - dt * 0.5f);
            if (c.pressure >= 6) endChase(true);
            else if (gChaseFarT > 7 || d > 450) endChase(false);
        }
    }
    // остановленная машина рядом / брошенная
    int stoppedNear = -1;
    for (size_t i = 0; i < CARS.size(); i++) {
        AICar& c = CARS[i];
        if (!c.alive || c.mode != CMODE_STOPPED || c.caseIdx < 0) continue;
        float d = dist2d(c.pos, PL.pos);
        if (d < 16) stoppedNear = (int)i;
        if (d > 130) { c.modeT += 0; if (c.modeT > 30) { c.mode = CMODE_LEAVE; c.modeT = 0; c.caseIdx = -1; c.checked = true; addRep(-1); toast("ВОДИТЕЛЬ НЕ ДОЖДАЛСЯ И УЕХАЛ. -1 РЕП.", 2); } }
    }
    gInspect = stoppedNear;
    // ориентировка
    if (SHF.minutes > SHF.nextWanted && gChaseCar < 0) { spawnWanted(); SHF.nextWanted = SHF.minutes + 120 + grng.i(90); }
    if (SHF.wantedCar >= 0 && (SHF.wantedCar >= (int)CARS.size() || !CARS[SHF.wantedCar].alive || !CARS[SHF.wantedCar].wanted)) SHF.wantedCar = -1;
    // доход камер
    while (SHF.minutes - SHF.lastCamMin >= 1.0f) {
        SHF.lastCamMin += 1.0f;
        int inc = 0;
        for (const Camera_& cam : SPEEDCAMS) inc += EDGES[cam.edge].highway ? 12 : 8;
        if (inc > 0) { SHF.camIncome += inc; SV.money += inc; }
    }
    // подсказки обучения
    if (SHF.tutorial) {
        gTutHintT -= dt;
        if (gTutHintT <= 0) {
            gTutHintT = 14;
            static const char* tips[] = {"ПОДСКАЗКА: РАДАР ВНИЗУ ПОКАЗЫВАЕТ СКОРОСТЬ МАШИНЫ ВПЕРЕДИ", "ПОДСКАЗКА: ЦЕЛЬ ПОДСВЕЧЕНА ЖЁЛТОЙ РАМКОЙ. «!» — НАРУШИТЕЛЬ",
                                         "ПОДСКАЗКА: СТОЯ НА МЕСТЕ, МОЖНО ОСТАНАВЛИВАТЬ МАШИНЫ ЖЕЗЛОМ", "ПОДСКАЗКА: ПОДЪЕДЬ К ОСТАНОВЛЕННОЙ МАШИНЕ И ПРОВЕРЬ ДОКУМЕНТЫ",
                                         "ПОДСКАЗКА: ПАМЯТКА С ПРАВИЛАМИ — В ПАУЗЕ И НА ЭКРАНЕ ДОКУМЕНТОВ"};
            if (SHF.tutStep < 5) toast(tips[SHF.tutStep++], 3, 6); else { SHF.tutorial = false; SV.tutorialShown = true; }
        }
    }
    // звук
    SY.engRpm = PL.rpm; SY.engLoad = PL.load; SY.engVol = 1; SY.siren = PL.siren ? 1.f : 0.f;
    SY.rain = L.rain; SY.ambient = districtAt(PL.pos) == 0 ? 0.25f : 0.12f;
    SY.screech = PL.sliding ? clampf(fabsf(PL.speed) / 20, 0, 1) : 0;
    if (SHF.minutes >= SHF.endMin) endShift();
}

// ---------------------------------------------------------------------
//  HUD
// ---------------------------------------------------------------------
static void drawIconArrow(float cx, float cy, float s, int dir, SDL_Color c) {
    if (dir < 0) UI.tri(cx - s, cy, cx + s * 0.6f, cy - s, cx + s * 0.6f, cy + s, c);
    else UI.tri(cx + s, cy, cx - s * 0.6f, cy - s, cx - s * 0.6f, cy + s, c);
}
static void drawMinimap(float x, float y, float sz) {
    panel(x, y, sz, sz, RGBA(10, 16, 26, 190));
    float scale = sz / 600.0f;
    float cx = x + sz / 2, cy = y + sz / 2;
    float yw = PL.yaw;
    auto tr = [&](const V3& p, float& ox, float& oy) {
        float dx = p.x - PL.pos.x, dz = p.z - PL.pos.z;
        // поворот так, чтобы направление движения было вверх
        float rx = dx * cosf(yw) - dz * sinf(yw), rz = dx * sinf(yw) + dz * cosf(yw);
        ox = cx + rx * scale; oy = cy - rz * scale;
    };
    // обрезка — грубо: рисуем только сегменты внутри
    for (const Edge& e : EDGES) {
        int ns = (int)(e.len / 40) + 1;
        V3 A = NODES[e.a].p, B = NODES[e.b].p;
        for (int k = 0; k < ns; k++) {
            V3 p0 = lerp3(A, B, k / (float)ns), p1 = lerp3(A, B, (k + 1) / (float)ns);
            float x0, y0, x1, y1; tr(p0, x0, y0); tr(p1, x1, y1);
            if (x0 < x + 3 || x0 > x + sz - 3 || y0 < y + 3 || y0 > y + sz - 3 || x1 < x + 3 || x1 > x + sz - 3 || y1 < y + 3 || y1 > y + sz - 3) continue;
            UI.line(x0, y0, x1, y1, e.highway ? 4 : 2.5f, e.highway ? RGBA(200, 200, 210) : RGBA(130, 140, 155));
        }
    }
    auto dot2 = [&](const V3& p, SDL_Color c, float r) {
        float ox, oy; tr(p, ox, oy);
        ox = clampf(ox, x + r, x + sz - r); oy = clampf(oy, y + r, y + sz - r);
        UI.circle(ox, oy, r, c, 10);
    };
    for (const Camera_& cam : SPEEDCAMS) { V3 p; float yy; camWorldPos(cam, p, yy); float ox, oy; tr(p, ox, oy); if (ox > x && ox < x + sz && oy > y && oy < y + sz) UI.rect(ox - 2, oy - 2, 4, 4, C_YELLOW); }
    for (size_t i = 0; i < CARS.size(); i++) {
        const AICar& c = CARS[i]; if (!c.alive) continue;
        if ((int)i == gChaseCar) dot2(c.pos, fmodf(gTime, 0.5f) < 0.25f ? C_RED : C_WHITE, 4.5f);
        else if (c.wanted) dot2(c.pos, RGBA(255, 120, 0), 4);
        else if (c.mode == CMODE_STOPPED && c.caseIdx >= 0) dot2(c.pos, C_YELLOW, 3.5f);
        else if (c.mode == CMODE_HELPER) dot2(c.pos, RGBA(80, 140, 255), 3.5f);
    }
    if (gBlock.active) dot2(gBlock.pos, RGBA(80, 140, 255), 5);
    float ox, oy; tr(POSTS[SHF.district].pos, ox, oy);
    if (ox > x && ox < x + sz && oy > y && oy < y + sz) UI.text(ox, oy - 3, "П", 1.2f * U() * 0.8f, RGBA(120, 200, 255), 1);
    UI.tri(cx, cy - 7, cx - 5, cy + 6, cx + 5, cy + 6, C_WHITE);
}

static void worldLabel(const V3& p, const char* s, SDL_Color c, float sc) {
    float sx, sy, sz;
    if (!R3.project(p, sx, sy, sz)) return;
    if (sx < 0 || sx > SW || sy < 0 || sy > SH) return;
    float w = UI.textW(s, sc) + 8 * sc / U();
    UI.roundRect(sx - w / 2, sy - 9 * sc, w, 11 * sc, 3, RGBA(0, 0, 0, 150));
    UI.text(sx, sy - 7.5f * sc, s, sc, c, 1);
}

static void drawDriveHUD() {
    float u = U();
    float mx = std::max(SW * 0.035f, 16.0f), mt = SH * 0.045f;
    // маркеры над машинами
    for (size_t i = 0; i < CARS.size(); i++) {
        const AICar& c = CARS[i]; if (!c.alive) continue;
        float d = dist2d(c.pos, PL.pos); if (d > 140) continue;
        V3 top = c.pos + V3(0, CM[c.model].height + 0.6f, 0);
        if ((int)i == gChaseCar) worldLabel(top, "БЕГЛЕЦ", C_RED, u * 1.1f);
        else if (c.mode == CMODE_STOPPED && c.caseIdx >= 0) worldLabel(top, "ЖДЁТ ПРОВЕРКИ", C_YELLOW, u * 0.9f);
        else if (c.wanted && d < 100) worldLabel(top, "РОЗЫСК", RGBA(255, 140, 0), u * 1.0f);
        else if (c.viol > 0 && d < 110 && c.mode == CMODE_DRIVE) worldLabel(top, c.viol == 3 ? "! КРАСНЫЙ" : fmt("! %d", (int)c.violSpeed).c_str(), RGBA(255, 90, 80), u * 1.0f);
    }
    if (gTarget >= 0) {
        const AICar& c = CARS[gTarget];
        float sx, sy, sz;
        if (R3.project(c.pos + V3(0, CM[c.model].height * 0.5f, 0), sx, sy, sz)) {
            float r = std::max(20.f, CM[c.model].radius * R3.focal / sz * 0.75f);
            SDL_Color yc = RGBA(255, 220, 60, 220);
            float L2 = r * 0.35f, t = std::max(2.f, u * 0.8f);
            for (int k = 0; k < 4; k++) {
                float sxk = (k & 1) ? 1.f : -1.f, syk = (k & 2) ? 1.f : -1.f;
                float px = sx + sxk * r, py = sy + syk * r * 0.65f;
                UI.rect(sxk > 0 ? px - L2 : px, py - t / 2, L2, t, yc);
                UI.rect(px - t / 2, syk > 0 ? py - L2 : py, t, L2, yc);
            }
        }
    }
    // верхняя левая панель
    float pw = u * 120, ph = u * 38;
    panel(mx, mt, pw, ph);
    int hh = (int)gHour, mm = (int)((gHour - hh) * 60);
    UI.text(mx + u * 5, mt + u * 4, fmt("%02d:%02d", hh, mm).c_str(), u * 1.4f, C_WHITE);
    UI.text(mx + pw - u * 5, mt + u * 5, fmt("%d ₽", SV.money).c_str(), u * 1.15f, C_YELLOW, 2);
    UI.text(mx + u * 5, mt + u * 18, RANKS[rankOf(SV.rep)], u * 0.95f, C_LIGHT);
    UI.text(mx + u * 5, mt + u * 28, fmt("РЕПУТАЦИЯ %d", SV.rep).c_str(), u * 0.9f, RGBA(150, 200, 255));
    drawMinimap(mx, mt + ph + u * 4, u * 74);
    // задания
    float tx = mx + pw + u * 8, tw = u * 150;
    panel(tx, mt, tw, u * 34, RGBA(18, 26, 40, 170));
    for (int i = 0; i < 3; i++) {
        const Task& t = SHF.tasks[i];
        UI.text(tx + u * 5, mt + u * (4 + i * 10), taskText(t), u * 0.9f, t.done ? RGBA(110, 220, 120) : C_WHITE);
    }
    // кнопки справа сверху
    float bs = u * 30;
    float bx = SW - mx - bs;
    if (button(bx, mt, bs, bs, "", C_GRAY, u * 1.6f) || gBackPressed) { gBackPressed = false; gScreen = SC_PAUSE; }
    UI.rect(bx + bs * 0.32f, mt + bs * 0.28f, bs * 0.12f, bs * 0.44f, C_WHITE); UI.rect(bx + bs * 0.56f, mt + bs * 0.28f, bs * 0.12f, bs * 0.44f, C_WHITE);
    if (button(bx - bs - u * 5, mt, bs, bs, gCamMode == 0 ? "ВИД" : "ВИД", C_GRAY, u * 1.0f)) { gCamMode = 1 - gCamMode; SV.camView = gCamMode; updateCamera(0, true); }
    if (button(bx - 2 * (bs + u * 5), mt, bs, bs, "КАРТА", C_GRAY, u * 0.8f)) { gMapReturn = SC_DRIVE; gScreen = SC_MAP; TOASTS.clear(); }
    // сообщения
    float my = mt + u * 40 + (SHF.wantedCar >= 0 ? u * 6 : 0);
    for (size_t i = 0; i < TOASTS.size(); i++) {
        const Toast& t = TOASTS[i];
        SDL_Color bg = t.kind == 1 ? RGBA(30, 110, 50, 220) : (t.kind == 2 ? RGBA(140, 30, 30, 220) : (t.kind == 3 ? RGBA(25, 60, 130, 220) : RGBA(20, 25, 35, 210)));
        float s = u * 1.05f; float tw0 = UI.textW(t.text.c_str(), s); if (tw0 > SW * 0.52f) { s *= SW * 0.52f / tw0; tw0 = SW * 0.52f; } float w = tw0 + u * 12;
        float a = clampf(t.t * 2, 0, 1);
        bg.a = (Uint8)(bg.a * a);
        UI.roundRect(SW / 2 - w / 2, my, w, u * 13, u * 3, bg);
        UI.text(SW / 2, my + u * 3, t.text.c_str(), s, RGBA(255, 255, 255, (int)(255 * a)), 1);
        my += u * 15;
    }
    if (gSubtitleT > 0) {
        float s = u * 1.1f; float w = UI.textW(gSubtitle.c_str(), s) + u * 10;
        UI.roundRect(SW / 2 - w / 2, SH * 0.62f, w, u * 13, u * 3, RGBA(0, 0, 0, 170));
        UI.text(SW / 2, SH * 0.62f + u * 3, gSubtitle.c_str(), s, C_YELLOW, 1);
    }
    // ориентировка
    if (SHF.wantedCar >= 0) {
        std::string s = "РОЗЫСК: " + SHF.wantedDesc;
        float w = UI.textW(s.c_str(), u * 0.9f) + u * 8;
        float wy = mt + bs + u * 5;
        float sc2 = u * 0.8f; w = UI.textW(s.c_str(), sc2) + u * 8;
        UI.roundRect(SW - mx - w, wy, w, u * 10, u * 2, RGBA(150, 70, 0, 200));
        UI.text(SW - mx - w / 2, wy + u * 1.8f, s.c_str(), sc2, C_WHITE, 1);
    }
    // спидометр и радар
    float sw = u * 84, sh = u * 40, sx = SW / 2 - sw / 2, sy = SH - sh - u * 6;
    panel(sx, sy, sw, sh, RGBA(10, 14, 22, 200));
    int kmh = (int)(fabsf(PL.speed) * 3.6f + 0.5f);
    bool over = kmh > PL.limitKmh + 5;
    UI.text(sx + u * 34, sy + u * 5, fmt("%d", kmh).c_str(), u * 2.6f, over ? RGBA(255, 120, 100) : C_WHITE, 1);
    UI.text(sx + u * 34, sy + u * 26, "КМ/Ч", u * 0.9f, C_LIGHT, 1);
    // знак ограничения
    float lx = sx + sw - u * 18, ly = sy + u * 17;
    UI.circle(lx, ly, u * 12, RGBA(210, 30, 30), 20); UI.circle(lx, ly, u * 9.3f, C_WHITE, 20);
    UI.text(lx, ly - u * 3.3f, fmt("%d", (int)PL.limitKmh).c_str(), u * 0.95f, RGBA(0, 0, 0), 1);
    if (PL.reverse) UI.text(sx + u * 6, sy + u * 5, "R", u * 1.4f, C_YELLOW);
    // радар
    float rw = u * 70, rx = sx - rw - u * 6;
    panel(rx, sy, rw, sh, RGBA(10, 14, 22, 200));
    UI.text(rx + rw / 2, sy + u * 4, "РАДАР", u * 0.9f, RGBA(120, 200, 255), 1);
    if (gRadarSpeed >= 0) {
        bool ov = gRadarSpeed > gRadarLimit + 20;
        UI.text(rx + rw / 2, sy + u * 14, fmt("%d", (int)gRadarSpeed).c_str(), u * 2.0f, ov ? RGBA(255, 90, 80) : RGBA(120, 255, 140), 1);
        UI.text(rx + rw / 2, sy + u * 30, fmt("ЛИМИТ %d", (int)gRadarLimit).c_str(), u * 0.8f, C_LIGHT, 1);
    } else UI.text(rx + rw / 2, sy + u * 17, "---", u * 1.6f, RGBA(90, 100, 110), 1);
    // погоня
    if (gChaseCar >= 0) {
        const AICar& c = CARS[gChaseCar];
        float d = dist2d(c.pos, PL.pos);
        float cw = u * 90, cx = sx + sw + u * 6;
        panel(cx, sy, cw, sh, RGBA(90, 15, 15, 210));
        UI.text(cx + cw / 2, sy + u * 4, "ПОГОНЯ", u * 1.1f, C_WHITE, 1);
        UI.text(cx + cw / 2, sy + u * 15, fmt("%d М", (int)d).c_str(), u * 1.2f, d > 200 ? C_YELLOW : C_WHITE, 1);
        UI.rect(cx + u * 6, sy + u * 29, cw - u * 12, u * 5, RGBA(0, 0, 0, 160));
        UI.rect(cx + u * 6, sy + u * 29, (cw - u * 12) * clampf(c.pressure / 6, 0, 1), u * 5, C_YELLOW);
    }
    // управление: руль
    float b = SH * 0.2f;
    float by = SH - b - SH * 0.04f;
    float lbx = mx, rbx = mx + b * 1.12f;
    bool lh = heldIn(lbx - mx, by - b * 0.3f, b + mx + b * 0.06f, b * 1.4f), rh = heldIn(rbx - b * 0.06f, by - b * 0.3f, b * 1.2f, b * 1.4f);
    UI.roundRect(lbx, by, b, b, b * 0.2f, lh ? RGBA(255, 255, 255, 90) : RGBA(255, 255, 255, 45));
    UI.roundRect(rbx, by, b, b, b * 0.2f, rh ? RGBA(255, 255, 255, 90) : RGBA(255, 255, 255, 45));
    drawIconArrow(lbx + b / 2, by + b / 2, b * 0.25f, -1, RGBA(255, 255, 255, 220));
    drawIconArrow(rbx + b / 2, by + b / 2, b * 0.25f, 1, RGBA(255, 255, 255, 220));
    // педали
    float gw = b * 0.82f, gh = b * 1.35f;
    float gx = SW - mx - gw, gy = SH - gh - SH * 0.04f;
    float bw = b * 0.95f, bh = b * 0.9f, bkx = gx - bw - b * 0.12f, bky = SH - bh - SH * 0.04f;
    bool gas = heldIn(gx - b * 0.06f, gy - b * 0.1f, gw + mx + b * 0.06f, gh + SH * 0.04f + b * 0.1f);
    bool brk = heldIn(bkx - b * 0.06f, bky - b * 0.1f, bw + b * 0.15f, bh + SH * 0.04f + b * 0.1f);
    UI.roundRect(gx, gy, gw, gh, b * 0.15f, gas ? RGBA(60, 200, 90, 160) : RGBA(60, 200, 90, 80));
    for (int k = 0; k < 4; k++) UI.rect(gx + gw * 0.2f, gy + gh * (0.2f + k * 0.17f), gw * 0.6f, gh * 0.06f, RGBA(255, 255, 255, 120));
    UI.text(gx + gw / 2, gy + gh - u * 12, "ГАЗ", u * 1.1f, C_WHITE, 1);
    UI.roundRect(bkx, bky, bw, bh, b * 0.15f, brk ? RGBA(220, 60, 60, 160) : RGBA(220, 60, 60, 80));
    for (int k = 0; k < 3; k++) UI.rect(bkx + bw * 0.15f, bky + bh * (0.2f + k * 0.2f), bw * 0.7f, bh * 0.08f, RGBA(255, 255, 255, 110));
    UI.text(bkx + bw / 2, bky + bh - u * 12, "ТОРМОЗ", u * 1.0f, C_WHITE, 1);
    // кнопки действий
    float ab = b * 0.72f;
    float ax = SW - mx - ab, ay = gy - ab - b * 0.18f;
    // мигалка
    {
        bool on = PL.flash;
        SDL_Color c = on ? (fmodf(gTime, 0.5f) < 0.25f ? RGBA(200, 40, 40) : RGBA(40, 80, 220)) : RGBA(50, 60, 80);
        if (button(ax, ay, ab, ab * 0.8f, "МИГАЛКА", c, u * 0.85f)) { PL.flash = !PL.flash; PL.siren = PL.flash; sfx(SFX_BEEP, 0.5f, PL.flash ? 1200 : 600); }
    }
    {
        bool parked = fabsf(PL.speed) < 1.5f;
        bool en = gTarget >= 0 && gDemandCooldown <= 0;
        if (button(ax - ab - b * 0.12f, ay, ab, ab * 0.8f, parked ? "ЖЕЗЛ" : "ТРЕБОВАТЬ", en ? RGBA(200, 140, 20) : RGBA(70, 70, 70), u * 0.85f, en)) demandStop();
    }
    if (gChaseCar >= 0) {
        float ry = ay - ab * 0.8f - b * 0.1f;
        if (button(ax, ry, ab, ab * 0.7f, "ПОМОЩЬ", C_BLUE, u * 0.85f, !gHelpUsed)) requestHelp();
        if (button(ax - ab - b * 0.12f, ry, ab, ab * 0.7f, "БЛОКПОСТ", C_BLUE, u * 0.85f, !gBlockUsed)) requestRoadblock();
    }
    // контекстная кнопка: проверить документы
    if (gInspect >= 0 && fabsf(PL.speed) < 2.0f) {
        float cw = u * 150, ch = u * 24;
        if (button(SW / 2 - cw / 2, sy - ch - u * 8, cw, ch, "ПРОВЕРИТЬ ДОКУМЕНТЫ", C_GREEN, u * 1.3f)) {
            gScreen = SC_DOCS; sfx(SFX_THUNK);
            PL.speed = 0;
            Case& c = CASES[CARS[gInspect].caseIdx];
            sfx(SFX_SPEECH, 0.8f, c.female ? 210 : 120, 0, 1.8f);
        }
    } else if (gInspect >= 0) {
        UI.text(SW / 2, sy - u * 18, "ОСТАНОВИСЬ РЯДОМ С МАШИНОЙ", u * 1.0f, C_YELLOW, 1, true);
    }
    // ввод
    gIn.steer = (rh ? 1.f : 0.f) - (lh ? 1.f : 0.f);
    gIn.throttle = gas ? 1.f : 0.f; gIn.brake = brk ? 1.f : 0.f;
}

static void drawCockpitOverlay() {
    float u = U();
    // стойки и приборная панель
    SDL_Color dark = RGBA(22, 22, 26);
    UI.quad(0, 0, SW * 0.07f, 0, SW * 0.13f, SH * 0.62f, 0, SH * 0.66f, dark);
    UI.quad(SW, 0, SW * 0.93f, 0, SW * 0.87f, SH * 0.62f, SW, SH * 0.66f, dark);
    UI.rect(0, 0, SW, SH * 0.06f, dark);
    UI.quad(0, SH * 0.66f, SW, SH * 0.66f, SW, SH, 0, SH, RGBA(30, 30, 34));
    UI.rectG(0, SH * 0.66f, SW, SH * 0.05f, RGBA(45, 45, 50), RGBA(30, 30, 34));
    // зеркало
    UI.roundRect(SW * 0.42f, SH * 0.07f, SW * 0.16f, SH * 0.07f, u * 3, RGBA(15, 15, 18));
    UI.roundRect(SW * 0.425f, SH * 0.078f, SW * 0.15f, SH * 0.055f, u * 2, toSDL(L.skyHor * 0.6f));
    // руль
    float cx = SW * 0.32f, cy = SH * 0.95f, r = SH * 0.3f;
    UI.ring(cx, cy, r * 0.86f, r, RGBA(15, 15, 17), 40);
    float a = -PL.steer * 2.6f;
    for (int k = 0; k < 3; k++) {
        float an = a + PI * (1.5f + k * 2.0f / 3.0f);
        UI.line(cx, cy, cx + cosf(an) * r * 0.88f, cy + sinf(an) * r * 0.88f, r * 0.12f, RGBA(20, 20, 23));
    }
    UI.circle(cx, cy, r * 0.22f, RGBA(28, 28, 32));
    // приборы
    float gx = SW * 0.32f, gy = SH * 0.76f;
    UI.circle(gx, gy - r * 0.15f, SH * 0.075f, RGBA(8, 10, 14), 24);
    float sp = fabsf(PL.speed) * 3.6f;
    float an = PI * 0.75f + clampf(sp / 240, 0, 1) * PI * 1.5f;
    UI.line(gx, gy - r * 0.15f, gx + cosf(an) * SH * 0.065f, gy - r * 0.15f + sinf(an) * SH * 0.065f, 3, RGBA(255, 80, 40));
    // дворники в дождь
    if (L.rain > 0) {
        float t = fmodf(gTime * 0.9f, 2.0f); float ph = t < 1 ? t : 2 - t;
        float wa = PI * (1.05f + ph * 0.75f);
        for (int k = 0; k < 2; k++) {
            float px = SW * (0.3f + k * 0.38f), py = SH * 0.66f;
            UI.line(px, py, px + cosf(wa) * SH * 0.55f, py + sinf(wa) * SH * 0.55f, u * 2.2f, RGBA(10, 10, 10));
        }
    }
}

// ---------------------------------------------------------------------
//  Экран документов
// ---------------------------------------------------------------------
static void docField(float x, float& y, float u, const char* label, const std::string& val, SDL_Color vc = RGBA(20, 20, 30)) {
    UI.text(x, y, label, u * 0.8f, RGBA(90, 80, 90));
    UI.text(x + UI.textW(label, u * 0.8f) + u * 3, y - u * 0.3f, val.c_str(), u * 1.0f, vc);
    y += u * 10;
}
static void drawDocs(float dt) {
    float u = U();
    AICar& car = CARS[gInspect];
    Case& c = CASES[car.caseIdx];
    UI.rect(0, 0, SW, SH, RGBA(0, 0, 0, 150));
    float mx = std::max(SW * 0.035f, 16.0f), mt = SH * 0.04f;
    float W = SW - mx * 2;
    // верхняя строка
    panel(mx, mt, W, u * 20, C_DARK);
    std::string top = fmt("СЕГОДНЯ: %s    АВТОМОБИЛЬ: %s, %s, %s %d", dateStr(TODAY()).c_str(), CM[car.model].name.c_str(), car.taxi ? "ЖЁЛТЫЙ" : colorName(car.paint), car.plateStr.c_str(), car.region);
    UI.text(mx + u * 6, mt + u * 6, top.c_str(), u * 1.05f, C_WHITE);
    float y0 = mt + u * 24;
    float colW = W * 0.25f;
    // водитель
    panel(mx, y0, colW, SH * 0.62f, C_DARK);
    UI.text(mx + u * 5, y0 + u * 4, "ВОДИТЕЛЬ", u * 1.0f, RGBA(150, 200, 255));
    float fw = colW - u * 10, fh = SH * 0.3f;
    drawFace(mx + u * 5, y0 + u * 14, fw, fh, c.face, c.female, c.age, true, c.belt, c.nervous);
    float ty = y0 + u * 16 + fh;
    UI.textBox(mx + u * 5, ty, fw, ("«" + c.speech + "»").c_str(), u * 0.85f, C_WHITE);
    // причина
    static const char* reasons[] = {"ПРОВЕРКА ДОКУМЕНТОВ", "ПРЕВЫШЕНИЕ СКОРОСТИ", "ПРЕВЫШЕНИЕ СКОРОСТИ", "ПРОЕЗД НА КРАСНЫЙ"};
    std::string rs = std::string("ПРИЧИНА: ") + reasons[c.reason];
    if (c.reason == 1 || c.reason == 2) rs += fmt(" (%d ПРИ %d)", (int)c.violSpeed, (int)c.violLimit);
    if (c.fled) rs = "ПРИЧИНА: ПОПЫТКА СКРЫТЬСЯ";
    panel(mx, y0 + SH * 0.62f + u * 4, colW, u * 22, RGBA(60, 20, 20, 220));
    UI.textBox(mx + u * 5, y0 + SH * 0.62f + u * 8, colW - u * 10, rs.c_str(), u * 0.85f, C_WHITE);
    // ВУ
    float cx = mx + colW + u * 6, cw = W * 0.37f, ch = SH * 0.42f;
    UI.roundRect(cx, y0, cw, ch, u * 5, RGBA(245, 220, 225));
    UI.rectG(cx, y0, cw, u * 14, RGBA(220, 150, 170), RGBA(235, 190, 200));
    UI.text(cx + cw / 2, y0 + u * 4, c.lForged ? "ВОДИТЕЛСКОЕ УДОСТОВЕРЕНИЕ" : "ВОДИТЕЛЬСКОЕ УДОСТОВЕРЕНИЕ", u * 0.95f, RGBA(80, 20, 40), 1);
    float pw = cw * 0.3f, ph = ch * 0.62f;
    drawFace(cx + u * 6, y0 + u * 18, pw, ph, c.lFace, c.female, c.age + (c.lFace != c.face ? 0 : -3), false, false, false);
    float fx = cx + pw + u * 12, fy = y0 + u * 18;
    docField(fx, fy, u, "1.", c.lSur);
    docField(fx, fy, u, "2.", c.lName + " " + c.lPatr);
    docField(fx, fy, u, "3.", dateStr(c.lBdate));
    docField(fx, fy, u, "4А.", dateStr(c.lIssue));
    docField(fx, fy, u, "4В.", dateStr(c.lValid), c.lValid < TODAY() ? RGBA(20, 20, 30) : RGBA(20, 20, 30));
    docField(fx, fy, u, "5.", c.lNum);
    docField(fx, fy, u, "9.", c.lCats);
    UI.text(cx + u * 6, y0 + ch - u * 12, "RUS", u * 1.2f, RGBA(80, 20, 40));
    if (!c.lForged) {
        float hx = cx + cw - u * 26, hy = y0 + ch - u * 24;
        for (int k = 0; k < 6; k++) { float t = fmodf(gTime * 0.3f + k / 6.0f, 1.0f); UI.circle(hx + u * 9, hy + u * 9, u * (9 - k * 1.3f), toSDL(Col(0.5f + 0.5f * sinf(t * 6.28f), 0.5f + 0.5f * sinf(t * 6.28f + 2), 0.5f + 0.5f * sinf(t * 6.28f + 4)), 0.6f), 16); }
    } else {
        UI.text(cx + cw - u * 30, y0 + ch - u * 14, "— — —", u * 0.9f, RGBA(160, 140, 150));
    }
    // СТС
    float sy = y0 + ch + u * 6, sh = SH * 0.62f + u * 26 - ch - u * 6;
    UI.roundRect(cx, sy, cw, sh, u * 5, RGBA(220, 238, 220));
    UI.rectG(cx, sy, cw, u * 13, RGBA(140, 190, 140), RGBA(190, 220, 190));
    UI.text(cx + cw / 2, sy + u * 3.5f, "СВИДЕТЕЛЬСТВО О РЕГИСТРАЦИИ ТС", u * 0.9f, RGBA(20, 60, 20), 1);
    float gy = sy + u * 17;
    docField(cx + u * 6, gy, u, "РЕГ. ЗНАК:", fmt("%s %d", c.sPlate.c_str(), c.sRegion));
    docField(cx + u * 6, gy, u, "МАРКА, МОДЕЛЬ:", c.sModel);
    docField(cx + u * 6, gy, u, "ЦВЕТ:", c.sColor);
    docField(cx + u * 6, gy, u, "СОБСТВЕННИК:", c.oSur + " " + firstLetter(c.oName) + "." + firstLetter(c.oPatr) + ".");
    docField(cx + u * 6, gy, u, "СЕРИЯ, №:", c.sNum);
    // ОСАГО
    float ox = cx + cw + u * 6, ow = W - colW - cw - u * 12, oh = SH * 0.32f;
    if (c.osPresent) {
        UI.roundRect(ox, y0, ow, oh, u * 5, RGBA(235, 240, 250));
        UI.rectG(ox, y0, ow, u * 13, RGBA(110, 150, 210), RGBA(170, 195, 235));
        UI.text(ox + ow / 2, y0 + u * 3.5f, "ПОЛИС ОСАГО", u * 1.0f, RGBA(10, 30, 80), 1);
        float oy = y0 + u * 17;
        docField(ox + u * 5, oy, u, "№", c.osNum);
        docField(ox + u * 5, oy, u, "ДЕЙСТВУЕТ ДО:", dateStr(c.osValid));
        docField(ox + u * 5, oy, u, "ТС:", fmt("%s %d", c.osPlate.c_str(), c.osRegion));
        UI.text(ox + u * 5, oy, "ДОПУЩЕНЫ К УПРАВЛЕНИЮ:", u * 0.8f, RGBA(90, 80, 90)); oy += u * 9;
        UI.textBox(ox + u * 5, oy, ow - u * 10, c.osUnlimited ? "БЕЗ ОГРАНИЧЕНИЙ" : c.osDrivers.c_str(), u * 0.9f, RGBA(20, 20, 30));
    } else {
        UI.roundRect(ox, y0, ow, oh, u * 5, RGBA(70, 75, 85, 220));
        UI.textBox(ox + u * 8, y0 + oh / 2 - u * 8, ow - u * 16, "ПОЛИС ОСАГО НЕ ПРЕДЪЯВЛЕН: «ЗАБЫЛ ДОМА...»", u * 1.0f, C_WHITE);
    }
    // база данных
    float by = y0 + oh + u * 6, bh = SH * 0.62f + u * 26 - oh - u * 6;
    panel(ox, by, ow, bh, RGBA(5, 25, 15, 235));
    UI.text(ox + u * 5, by + u * 4, "БАЗА ДАННЫХ ГИБДД", u * 0.95f, RGBA(100, 255, 140));
    if (!c.baseChecked && c.baseT <= 0) {
        if (button(ox + u * 5, by + u * 16, ow - u * 10, u * 20, "ПРОБИТЬ ПО БАЗЕ", RGBA(20, 110, 60), u * 1.0f)) { c.baseT = 2.0f; sfx(SFX_RADIO, 0.5f, 0, 0, 1.6f); }
    } else if (!c.baseChecked) {
        c.baseT -= dt;
        int dots = (int)(gTime * 3) % 4;
        UI.text(ox + u * 5, by + u * 18, (std::string("ЗАПРОС") + std::string(dots, '.')).c_str(), u * 1.0f, RGBA(100, 255, 140));
        if (((int)(gTime * 20)) % 3 == 0) sfx(SFX_TYPE, 0.3f);
        if (c.baseT <= 0) c.baseChecked = true;
    } else {
        float ly = by + u * 16; float lw = ow - u * 10;
        SDL_Color g = RGBA(120, 255, 150), r = RGBA(255, 110, 100);
        std::string l1 = c.lForged ? "ВУ: НЕ НАЙДЕНО В БАЗЕ!" : (c.deprived ? "ВУ: ЛИШЁН ПРАВ ДО " + dateStr(c.deprivedUntil) : "ВУ " + c.lNum + ": " + c.lSur);
        ly += UI.textBox(ox + u * 5, ly, lw, l1.c_str(), u * 0.85f, (c.lForged || c.deprived) ? r : g) + u * 2;
        std::string l2 = c.plateMismatch ? std::string("НОМЕР ") + car.plateStr + ": ЗАРЕГИСТРИРОВАН НА ДРУГОЕ ТС!" : std::string("НОМЕР ") + car.plateStr + ": " + c.sModel;
        ly += UI.textBox(ox + u * 5, ly, lw, l2.c_str(), u * 0.85f, c.plateMismatch ? r : g) + u * 2;
        std::string l3 = c.wanted ? "РОЗЫСК: АВТОМОБИЛЬ В УГОНЕ!" : "РОЗЫСК: НЕ ЧИСЛИТСЯ";
        UI.textBox(ox + u * 5, ly, lw, l3.c_str(), u * 0.85f, c.wanted ? r : g);
    }
    // нижние кнопки
    float bh2 = u * 26, bw = (W - u * 12) / 3, byb = SH - bh2 - SH * 0.04f;
    if (button(mx, byb, bw, bh2, "ОСМОТР БАГАЖНИКА", C_GRAY, u * 1.1f)) { gScreen = SC_TRUNK; c.trunkOpened = true; sfx(SFX_THUNK); }
    if (button(mx + bw + u * 6, byb, bw, bh2, "ПАМЯТКА", C_GRAY, u * 1.1f)) { gHelpReturn = SC_DOCS; gScreen = SC_HELP; }
    if (button(mx + 2 * (bw + u * 6), byb, bw, bh2, "ПРИНЯТЬ РЕШЕНИЕ", C_BLUE, u * 1.1f)) { gScreen = SC_DECIDE; }
    if (gBackPressed) { gBackPressed = false; gScreen = SC_DRIVE; }
}

// ---------------------------------------------------------------------
//  Багажник
// ---------------------------------------------------------------------
static int gTrunkSel = -1;
static void trunkCamera(const AICar& car, int model) {
    const CarModel& M = CM[model];
    V3 pos = car.trailer ? car.tpos : car.pos; float yaw = car.trailer ? car.tyaw : car.yaw;
    V3 f = fwdOf(yaw);
    V3 rearPt = pos + f * M.trunkZ + V3(0, M.trunkY * 0.85f, 0);
    float back = M.tallRear ? 3.4f : 2.6f;
    V3 cam = pos + f * (M.minZ - back) + V3(0, M.trunkY + (M.tallRear ? 0.3f : 1.0f), 0);
    V3 d = rearPt - cam;
    R3.setCamera(cam, atan2f(d.x, d.z), atan2f(d.y, sqrtf(d.x * d.x + d.z * d.z)), 62);
}
static void drawTrunkInterior(const AICar& car, int model, Case& c) {
    const CarModel& M = CM[model];
    V3 pos = car.trailer ? car.tpos : car.pos; float yaw = car.trailer ? car.tyaw : car.yaw;
    Mat3 rot = rotYPR(yaw, 0, 0);
    auto W = [&](V3 p) { return mul(rot, p) + pos; };
    bool big = M.tallRear;
    float floorY = big ? (model == MD_TRUCK || model == MD_TRAILER ? 1.25f : 0.62f) : M.trunkY - 0.42f;
    float topY = big ? M.trunkY - 0.05f : M.trunkY;
    float z0 = M.minZ + 0.02f, z1 = M.trunkZ + (big ? 1.2f : 0.85f);
    if (model == MD_TRUCK) z1 = M.trunkZ + 2.2f;
    if (model == MD_TRAILER) z1 = M.trunkZ + 3.0f;
    if (model == MD_PICKUP) { floorY = 0.95f; z1 = M.trunkZ + 1.4f; topY = 1.3f; }
    float w = M.trunkW + 0.05f;
    Col carpet(0.14f, 0.14f, 0.16f), wall(0.2f, 0.2f, 0.22f);
    auto q = [&](V3 a, V3 b, V3 cc, V3 d, Col col) {
        V3 n = norm(cross(W(b) - W(a), W(d) - W(a)));
        R3.quadW(W(a), W(b), W(cc), W(d), toSDL(shadeN(col, n, W(a)) + col * 0.3f), 2, 0, 0);
    };
    q(V3(-w, floorY, z0), V3(w, floorY, z0), V3(w, floorY, z1), V3(-w, floorY, z1), carpet);
    q(V3(-w, floorY, z1), V3(w, floorY, z1), V3(w, topY, z1), V3(-w, topY, z1), wall);
    q(V3(-w, floorY, z0), V3(-w, floorY, z1), V3(-w, topY, z1), V3(-w, topY, z0), wall * 0.85f);
    q(V3(w, floorY, z0), V3(w, topY, z0), V3(w, topY, z1), V3(w, floorY, z1), wall * 0.85f);
    // крышка/дверь
    Col lid = car.paint;
    if (big) q(V3(-w - 0.05f, topY + 0.05f, z0 + 0.25f), V3(w + 0.05f, topY + 0.05f, z0 + 0.25f), V3(w + 0.05f, topY + 0.25f, z0 - 0.75f), V3(-w - 0.05f, topY + 0.25f, z0 - 0.75f), lid);
    else q(V3(-w - 0.03f, topY + 0.02f, z1), V3(w + 0.03f, topY + 0.02f, z1), V3(w + 0.03f, topY + 0.85f, z1 - 0.35f), V3(-w - 0.03f, topY + 0.85f, z1 - 0.35f), lid);
    R3.draw();
    // предметы
    for (size_t i = 0; i < c.items.size(); i++) {
        TrunkItem& it = c.items[i];
        Col col = it.col;
        if ((int)i == gTrunkSel) col = col + Col(0.25f, 0.22f, 0.05f);
        V3 cpos = W(it.lp);
        if (it.kind == 6) {
            DrawCar d; d.pos = cpos; d.yaw = yaw; d.spin = 0; d.steer = 0;
            WheelDef wd; wd.p = V3(0, 0, 0); wd.r = it.hs.x; wd.w = it.hs.y * 2; wd.steer = false;
            Mat3 rr = rotYPR(yaw + PI / 2, 0, PI / 2);
            drawWheel(wd, rr, cpos, yaw, d, 0, false);
        } else {
            shadedBox(cpos, it.hs, yaw, col);
            if (it.kind == 2) shadedBox(cpos + V3(0, it.hs.y + 0.03f, 0), V3(0.02f, 0.03f, it.hs.z * 0.6f), yaw, col * 0.6f);
            if (it.kind == 3 || it.kind == 0) shadedBox(cpos + V3(0, it.hs.y + 0.04f, 0), V3(it.hs.x * 0.4f, 0.03f, 0.02f), yaw, Col(0.1f, 0.1f, 0.1f));
            if (it.kind == 1) shadedBox(cpos + V3(0, it.hs.y + 0.002f, 0), V3(0.03f, 0.003f, it.hs.z), yaw, Col(0.75f, 0.65f, 0.4f));
        }
    }
    R3.draw();
}
static void drawTrunkScreen(float dt) {
    (void)dt;
    float u = U();
    AICar& car = CARS[gInspect];
    Case& c = CASES[car.caseIdx];
    int model = car.trailer ? MD_TRAILER : car.model;
    trunkCamera(car, model);
    drawWorldScene(dt, true, false);
    drawTrunkInterior(car, model, c);
    // выбор предмета
    for (Tap& t : TAPS) {
        if (t.used) continue;
        if (t.y > SH - u * 40) continue;
        float best = 1e9f; int bi = -1;
        Mat3 rot = rotYPR(car.trailer ? car.tyaw : car.yaw, 0, 0);
        V3 pos = car.trailer ? car.tpos : car.pos;
        for (size_t i = 0; i < c.items.size(); i++) {
            float sx, sy, sz;
            if (!R3.project(mul(rot, c.items[i].lp) + pos, sx, sy, sz)) continue;
            float r = std::max(c.items[i].hs.x, c.items[i].hs.z) * R3.focal / sz + u * 12;
            float d = sqrtf((sx - t.x) * (sx - t.x) + (sy - t.y) * (sy - t.y));
            if (d < r && d < best) { best = d; bi = (int)i; }
        }
        if (bi >= 0) { t.used = true; gTrunkSel = bi; c.items[bi].inspected = true; sfx(SFX_THUNK, 0.6f); if (c.items[bi].illegal) sfx(SFX_ERROR, 0.4f); }
    }
    for (size_t i = 0; i < c.items.size(); i++) {
        const TrunkItem& it = c.items[i];
        Mat3 rot = rotYPR(car.trailer ? car.tyaw : car.yaw, 0, 0);
        V3 pos = car.trailer ? car.tpos : car.pos;
        V3 wp = mul(rot, it.lp + V3(0, it.hs.y + 0.12f, 0)) + pos;
        std::string s = it.inspected ? std::string(ITEM_NAMES[it.kind]) + ": " + it.content : std::string(ITEM_NAMES[it.kind]) + " (?)";
        float sx, sy, sz;
        if (!R3.project(wp, sx, sy, sz)) continue;
        float sc = u * 1.0f, w = UI.textW(s.c_str(), sc) + u * 8, h = u * 11;
        // раздвигаем пересекающиеся подписи
        static std::vector<SDL_FRect> used; if (i == 0) used.clear();
        bool moved = true; int guard = 0;
        while (moved && guard++ < 10) {
            moved = false;
            for (const SDL_FRect& r : used) if (fabsf(r.x - sx) < (r.w + w) * 0.5f && fabsf(r.y - sy) < h) { sy = r.y - h - u; moved = true; }
        }
        SDL_FRect rr = {sx, sy, w, h}; used.push_back(rr);
        UI.roundRect(sx - w / 2, sy - 9 * sc, w, h, 3, RGBA(0, 0, 0, 160));
        UI.text(sx, sy - 7.5f * sc, s.c_str(), sc, it.illegal && it.inspected ? RGBA(255, 90, 80) : C_WHITE, 1);
    }
    float mx = std::max(SW * 0.035f, 16.0f);
    panel(mx, SH * 0.04f, SW - 2 * mx, u * 18, C_DARK);
    UI.text(SW / 2, SH * 0.04f + u * 5, "ОСМОТР БАГАЖНИКА: НАЖИМАЙ НА ПРЕДМЕТЫ, ЧТОБЫ ОСМОТРЕТЬ", u * 1.0f, C_WHITE, 1);
    if (button(SW / 2 - u * 70, SH - u * 30 - SH * 0.04f, u * 140, u * 26, "ЗАКРЫТЬ БАГАЖНИК", C_BLUE, u * 1.1f) || gBackPressed) { gBackPressed = false; gScreen = SC_DOCS; gTrunkSel = -1; sfx(SFX_THUNK); }
}

// ---------------------------------------------------------------------
//  Решение и результат
// ---------------------------------------------------------------------
static bool gArts[ART_N]; static bool gFineMode = false;
static std::string gResTitle; static std::vector<std::string> gResLines; static int gResMoney = 0, gResRep = 0; static bool gResGood = false;
static void drawDecide() {
    float u = U();
    UI.rect(0, 0, SW, SH, RGBA(0, 0, 0, 170));
    float w = u * 260, h = SH * 0.86f, x = SW / 2 - w / 2, y = SH * 0.07f;
    panel(x, y, w, h, C_DARK);
    UI.text(SW / 2, y + u * 6, gFineMode ? "ВЫБЕРИТЕ СТАТЬИ КОАП" : "РЕШЕНИЕ ПО ВОДИТЕЛЮ", u * 1.4f, C_WHITE, 1);
    float by = y + u * 24, bh = u * 25, bw = w - u * 20, bx = x + u * 10;
    if (!gFineMode) {
        SDL_Color cols[5] = {C_GREEN, RGBA(60, 120, 170), RGBA(200, 140, 20), RGBA(170, 90, 30), C_RED};
        for (int i = 0; i < 5; i++) {
            if (button(bx, by, bw, bh, VERDICT_NAMES[i], cols[i], u * 1.2f)) {
                if (i == V_FINE) { gFineMode = true; for (int k = 0; k < ART_N; k++) gArts[k] = false; }
                else { bool none[ART_N] = {false, false, false, false}; finishInspection(i, none, gResTitle, gResLines, gResMoney, gResRep, gResGood); gScreen = SC_RESULT; }
            }
            by += bh + u * 5;
        }
        if (button(bx, by + u * 4, bw, bh * 0.8f, "НАЗАД К ДОКУМЕНТАМ", C_GRAY, u * 1.0f) || gBackPressed) { gBackPressed = false; gScreen = SC_DOCS; }
    } else {
        for (int i = 0; i < ART_N; i++) {
            std::string l = std::string(gArts[i] ? "[X] " : "[ ] ") + ART_NAMES[i];
            if (button(bx, by, bw, bh, l.c_str(), gArts[i] ? RGBA(200, 140, 20) : C_GRAY, u * 1.0f)) gArts[i] = !gArts[i];
            by += bh + u * 5;
        }
        bool any = false; for (int i = 0; i < ART_N; i++) any |= gArts[i];
        if (button(bx, by + u * 4, bw, bh, "ВЫПИСАТЬ ШТРАФ", C_BLUE, u * 1.2f, any)) { finishInspection(V_FINE, gArts, gResTitle, gResLines, gResMoney, gResRep, gResGood); gScreen = SC_RESULT; gFineMode = false; }
        by += bh + u * 8;
        if (button(bx, by + u * 4, bw, bh * 0.8f, "НАЗАД", C_GRAY, u * 1.0f) || gBackPressed) { gBackPressed = false; gFineMode = false; }
    }
}
static void drawResult() {
    float u = U();
    UI.rect(0, 0, SW, SH, RGBA(0, 0, 0, 170));
    float w = u * 280, h = SH * 0.84f, x = SW / 2 - w / 2, y = SH * 0.08f;
    panel(x, y, w, h, C_DARK);
    UI.roundRect(x, y, w, u * 24, u * 5, gResGood ? RGBA(30, 120, 50) : RGBA(150, 30, 30));
    UI.text(SW / 2, y + u * 7, gResTitle.c_str(), u * 1.3f, C_WHITE, 1);
    float ly = y + u * 32;
    for (const std::string& l : gResLines) { ly += UI.textBox(x + u * 10, ly, w - u * 20, l.c_str(), u * 1.0f, C_LIGHT) + u * 1; }
    ly += u * 6;
    UI.text(x + u * 10, ly, fmt("ДЕНЬГИ: %+d ₽", gResMoney).c_str(), u * 1.3f, gResMoney >= 0 ? RGBA(120, 255, 140) : RGBA(255, 110, 100));
    UI.text(x + w / 2 + u * 10, ly, fmt("РЕПУТАЦИЯ: %+d", gResRep).c_str(), u * 1.3f, gResRep >= 0 ? RGBA(120, 255, 140) : RGBA(255, 110, 100));
    if (button(SW / 2 - u * 60, y + h - u * 32, u * 120, u * 24, "ДАЛЕЕ", C_BLUE, u * 1.2f) || gBackPressed) {
        gBackPressed = false; gScreen = SC_DRIVE; gInspect = -1;
        int rk = rankOf(SV.rep);
        static int lastRank = -1; if (lastRank < 0) lastRank = SHF.startRank;
        if (rk > lastRank) { toast(std::string("ПОВЫШЕНИЕ! НОВОЕ ЗВАНИЕ: ") + RANKS[rk], 1, 5); sfx(SFX_SUCCESS); }
        lastRank = rk;
    }
}

// ---------------------------------------------------------------------
//  Карта и камеры
// ---------------------------------------------------------------------
static int gMapConfirmCam = -1;
static void drawMap() {
    float u = U();
    UI.rect(0, 0, SW, SH, RGBA(14, 20, 30));
    float mx = std::max(SW * 0.035f, 16.0f), mt = SH * 0.04f;
    float x0 = -760, x1 = 860, z0 = -960, z1 = 1460;
    float areaW = SW * 0.66f, areaH = SH - mt * 2 - u * 6;
    float sc = std::min(areaW / (x1 - x0), areaH / (z1 - z0));
    float ox = mx + (areaW - (x1 - x0) * sc) / 2, oy = mt + (areaH - (z1 - z0) * sc) / 2;
    auto P = [&](const V3& p, float& sx, float& sy) { sx = ox + (p.x - x0) * sc; sy = oy + (z1 - p.z) * sc; };
    panel(mx - u * 3, mt - u * 3, areaW + u * 6, areaH + u * 6, RGBA(30, 45, 35, 255));
    // районы
    for (int d = 0; d < 4; d++) {
        V3 c = d == 0 ? V3(165, 0, 165) : (d == 1 ? V3(800, 0, -300) : (d == 2 ? V3(50, 0, 1400) : V3(-700, 0, -300)));
        float sx, sy; P(c, sx, sy);
        bool locked = rankOf(SV.rep) < DISTRICT_RANK[d];
        SDL_Color col = locked ? RGBA(200, 120, 120) : RGBA(200, 230, 200);
        float ty = d == 0 ? sy + u * 18 : (d == 2 ? sy - u * 16 : sy);
        float tx = d == 1 ? sx - u * 8 : (d == 3 ? sx + u * 8 : sx);
        UI.text(tx, ty, DISTRICT_NAMES[d], u * 0.85f, col, d == 1 ? 2 : (d == 3 ? 0 : 1), true);
        if (locked) UI.text(tx, ty + u * 9, fmt("С ЗВАНИЯ %s", RANKS[DISTRICT_RANK[d]]).c_str(), u * 0.7f, col, d == 1 ? 2 : (d == 3 ? 0 : 1));
    }
    for (const Edge& e : EDGES) {
        float ax, ay, bx, by; P(NODES[e.a].p, ax, ay); P(NODES[e.b].p, bx, by);
        UI.line(ax, ay, bx, by, e.highway ? u * 3.5f : u * 2.2f, e.highway ? RGBA(210, 210, 215) : RGBA(150, 155, 165));
    }
    for (int k = 0; k < 4; k++) { float sx, sy; P(POSTS[k].pos, sx, sy); UI.circle(sx, sy, u * 4, RGBA(60, 120, 255)); UI.text(sx, sy - u * 3, "П", u * 0.8f, C_WHITE, 1); }
    // камеры
    for (size_t i = 0; i < SPEEDCAMS.size(); i++) {
        V3 p; float y; camWorldPos(SPEEDCAMS[i], p, y); float sx, sy; P(p, sx, sy);
        UI.roundRect(sx - u * 4, sy - u * 4, u * 8, u * 8, u * 2, (int)i == gMapConfirmCam ? C_RED : C_YELLOW);
    }
    if (SHF.active) { float sx, sy; P(PL.pos, sx, sy); V3 f = fwdOf(PL.yaw); UI.circle(sx, sy, u * 4, C_WHITE); UI.line(sx, sy, sx + f.x * u * 9, sy - f.z * u * 9, u * 2, C_WHITE); }
    // правая панель
    float px = mx + areaW + u * 10, pw = SW - px - mx;
    panel(px, mt, pw, areaH, C_DARK);
    int slots = CAM_SLOTS[SV.upg[U_CAMS]];
    float ty = mt + u * 6;
    UI.text(px + u * 6, ty, "КАМЕРЫ СКОРОСТИ", u * 1.2f, C_WHITE); ty += u * 14;
    UI.text(px + u * 6, ty, fmt("УСТАНОВЛЕНО: %d / %d", (int)SPEEDCAMS.size(), slots).c_str(), u * 1.0f, C_YELLOW); ty += u * 12;
    int inc = 0; for (const Camera_& c : SPEEDCAMS) inc += EDGES[c.edge].highway ? 12 : 8;
    UI.text(px + u * 6, ty, fmt("ДОХОД: %d ₽ В ЧАС", inc * 60).c_str(), u * 1.0f, RGBA(120, 255, 140)); ty += u * 14;
    ty += UI.textBox(px + u * 6, ty, pw - u * 12, fmt("НАЖМИ НА ДОРОГУ, ЧТОБЫ ПОСТАВИТЬ КАМЕРУ ЗА %d ₽. НА ТРАССЕ КАМЕРА ПРИНОСИТ БОЛЬШЕ. НАЖМИ НА КАМЕРУ, ЧТОБЫ СНЯТЬ (ВОЗВРАТ ПОЛОВИНЫ).", CAMERA_PRICE).c_str(), u * 0.85f, C_LIGHT);
    ty += u * 6;
    UI.text(px + u * 6, ty, fmt("ДЕНЬГИ: %d ₽", SV.money).c_str(), u * 1.1f, C_YELLOW);
    if (gMapConfirmCam >= 0) {
        float by = mt + areaH - u * 64;
        UI.text(px + pw / 2, by - u * 12, "СНЯТЬ ЭТУ КАМЕРУ?", u * 1.0f, C_WHITE, 1);
        if (button(px + u * 6, by, pw - u * 12, u * 22, fmt("СНЯТЬ (+%d ₽)", CAMERA_PRICE / 2).c_str(), C_RED, u * 1.0f)) {
            SPEEDCAMS.erase(SPEEDCAMS.begin() + gMapConfirmCam); SV.money += CAMERA_PRICE / 2; gMapConfirmCam = -1; saveGame();
        }
    }
    if (button(px + u * 6, mt + areaH - u * 30, pw - u * 12, u * 24, "ГОТОВО", C_BLUE, u * 1.2f) || gBackPressed) { gBackPressed = false; gMapConfirmCam = -1; gScreen = gMapReturn; }
    // касания по карте
    for (Tap& t : TAPS) {
        if (t.used || t.x > mx + areaW || t.x < mx || t.y < mt || t.y > mt + areaH) continue;
        t.used = true;
        // камера под пальцем?
        int hitCam = -1;
        for (size_t i = 0; i < SPEEDCAMS.size(); i++) { V3 p; float y; camWorldPos(SPEEDCAMS[i], p, y); float sx, sy; P(p, sx, sy); if (fabsf(sx - t.x) < u * 9 && fabsf(sy - t.y) < u * 9) hitCam = (int)i; }
        if (hitCam >= 0) { gMapConfirmCam = hitCam; sfx(SFX_CLICK); continue; }
        gMapConfirmCam = -1;
        V3 wp(x0 + (t.x - ox) / sc, 0, z1 - (t.y - oy) / sc);
        RoadHit h = nearestRoad(wp);
        if (h.edge < 0 || h.dist * sc > u * 10) { toast("НАЖМИ ТОЧНЕЕ НА ДОРОГУ", 2, 2); continue; }
        const Edge& e = EDGES[h.edge];
        if (h.s < 20 || h.s > e.slen - 20) { toast("СЛИШКОМ БЛИЗКО К ПЕРЕКРЁСТКУ", 2, 2); continue; }
        if ((int)SPEEDCAMS.size() >= slots) { toast("НЕТ СВОБОДНЫХ КАМЕР. УЛУЧШИ В ГАРАЖЕ", 2, 2.5f); sfx(SFX_ERROR); continue; }
        if (SV.money < CAMERA_PRICE) { toast("НЕДОСТАТОЧНО ДЕНЕГ", 2, 2); sfx(SFX_ERROR); continue; }
        Camera_ c; c.edge = h.edge; c.dir = h.lat >= 0 ? 0 : 1; c.s = c.dir == 0 ? h.s : e.slen - h.s;
        SPEEDCAMS.push_back(c); SV.money -= CAMERA_PRICE; sfx(SFX_SHUTTER); toast("КАМЕРА УСТАНОВЛЕНА", 1, 2); saveGame();
    }
    // всплывающие сообщения
    float my = mt + u * 4;
    for (size_t i = 0; i < TOASTS.size(); i++) { const Toast& t = TOASTS[i]; float w = UI.textW(t.text.c_str(), u) + u * 10; UI.roundRect(mx + areaW / 2 - w / 2, my, w, u * 12, u * 2, RGBA(0, 0, 0, 200)); UI.text(mx + areaW / 2, my + u * 2.5f, t.text.c_str(), u, C_WHITE, 1); my += u * 14; }
}

// ---------------------------------------------------------------------
//  Гараж
// ---------------------------------------------------------------------
static int gGarageSel = 0; static float gGarageRot = 0;
static void drawGarage(float dt) {
    float u = U();
    gGarageRot += dt * 0.5f;
    // 3D-превью
    Lighting saved = L;
    updateLighting(15.5f, 0);
    L.fogStart = 200; L.fogEnd = 400; L.hlOn = 0;
    UI.rectG(0, 0, SW, SH, RGBA(40, 50, 70), RGBA(15, 18, 26));
    UI.flush();
    const CarModel& MM = CM[PATROL[gGarageSel].model];
    float camD = 6.5f + MM.halfL * 1.6f;
    V3 tgt(0, MM.height * 0.45f, 0);
    V3 cp(sinf(gGarageRot) * camD, 2.2f + MM.height * 0.5f, cosf(gGarageRot) * camD);
    V3 dd = tgt - cp;
    R3.setCamera(cp, atan2f(dd.x, dd.z), atan2f(dd.y, sqrtf(dd.x * dd.x + dd.z * dd.z)), 42);
    // смещение влево: камера смотрит немного вправо
    R3.cx = SW * 0.3f;
    // пол
    for (int i = 0; i < 24; i++) {
        float a0 = i * 2 * PI / 24, a1 = (i + 1) * 2 * PI / 24;
        R3.triW(V3(0, 0, 0), V3(cosf(a0) * 4.2f, 0, sinf(a0) * 4.2f), V3(cosf(a1) * 4.2f, 0, sinf(a1) * 4.2f), RGBA(60, 66, 78), 0);
    }
    const CarModel& M = CM[PATROL[gGarageSel].model];
    R3.groundSprite(V3(0, 0.02f, (M.maxZ + M.minZ) * 0.5f), V3(M.halfW * 1.4f, 0, 0), V3(0, 0, M.halfL * 1.2f), RGBA(0, 0, 0, 170), 0, 0);
    DrawCar d; d.pos = V3(0, 0, 0); d.yaw = 0; d.police = true; d.flash = true; d.time = gTime; d.paint = Col(0.95f, 0.95f, 0.96f); d.plate = 2; d.spin = gTime;
    drawCarModel(PATROL[gGarageSel].model, d);
    R3.draw();
    L = saved;
    float mx = std::max(SW * 0.035f, 16.0f), mt = SH * 0.04f;
    UI.text(mx, mt, "ГАРАЖ", u * 2.2f, C_WHITE, 0, true);
    UI.text(mx, mt + u * 20, fmt("%d ₽   %s", SV.money, RANKS[rankOf(SV.rep)]).c_str(), u * 1.1f, C_YELLOW, 0, true);
    const PatrolSpec& ps = PATROL[gGarageSel];
    float iy = SH * 0.76f;
    UI.text(SW * 0.3f, iy, ps.name, u * 1.8f, C_WHITE, 1, true);
    UI.text(SW * 0.3f, iy + u * 17, ps.desc, u * 0.85f, C_LIGHT, 1, true);
    UI.text(SW * 0.3f, iy + u * 28, fmt("МАКС. %d КМ/Ч    МАССА %.1f", (int)(ps.vmax * (1 + 0.07f * SV.upg[U_ENGINE])), ps.mass).c_str(), u * 0.9f, RGBA(150, 200, 255), 1, true);
    // список машин
    float px = SW * 0.58f, pw = SW - px - mx;
    float y = mt;
    for (int i = 0; i < 4; i++) {
        const PatrolSpec& s = PATROL[i];
        bool owned = (SV.carOwned >> i) & 1;
        std::string lbl = std::string(s.name) + (SV.car == i ? " (ВЫБРАНА)" : owned ? "" : fmt(" — %d ₽", s.price));
        if (button(px, y, pw, u * 20, lbl.c_str(), i == gGarageSel ? C_BLUE : C_GRAY, u * 1.0f)) gGarageSel = i;
        y += u * 23;
    }
    {
        const PatrolSpec& s = PATROL[gGarageSel];
        bool owned = (SV.carOwned >> gGarageSel) & 1;
        bool rankOk = rankOf(SV.rep) >= s.rank;
        if (owned) { if (SV.car != gGarageSel && button(px, y, pw, u * 20, "ВЫБРАТЬ", C_GREEN, u * 1.1f)) { SV.car = gGarageSel; saveGame(); } }
        else if (!rankOk) { UI.text(px + pw / 2, y + u * 6, fmt("НУЖНО ЗВАНИЕ: %s", RANKS[s.rank]).c_str(), u * 0.9f, RGBA(255, 130, 120), 1); }
        else if (button(px, y, pw, u * 20, fmt("КУПИТЬ ЗА %d ₽", s.price).c_str(), C_GREEN, u * 1.1f, SV.money >= s.price)) { SV.money -= s.price; SV.carOwned |= 1 << gGarageSel; SV.car = gGarageSel; sfx(SFX_COIN); saveGame(); }
        y += u * 26;
    }
    UI.text(px, y, "УЛУЧШЕНИЯ (ДЛЯ ВСЕХ МАШИН)", u * 1.0f, RGBA(150, 200, 255)); y += u * 12;
    for (int k = 0; k < U_COUNT; k++) {
        int lv = SV.upg[k];
        UI.text(px, y + u * 2, UPG_NAMES[k], u * 0.95f, C_WHITE);
        UI.text(px, y + u * 11, UPG_DESC[k], u * 0.7f, RGBA(150, 160, 175));
        for (int p = 0; p < 3; p++) UI.rect(px + pw * 0.45f + p * u * 9, y + u * 4, u * 7, u * 7, p < lv ? C_YELLOW : RGBA(60, 66, 76));
        if (lv < 3) { if (button(px + pw * 0.66f, y, pw * 0.34f, u * 17, fmt("%d ₽", UPG_PRICE[k][lv]).c_str(), C_GREEN, u * 0.9f, SV.money >= UPG_PRICE[k][lv])) { SV.money -= UPG_PRICE[k][lv]; SV.upg[k]++; sfx(SFX_COIN); saveGame(); } }
        else UI.text(px + pw * 0.83f, y + u * 5, "МАКС.", u * 0.9f, C_YELLOW, 1);
        y += u * 21;
    }
    if (button(mx, SH - mt - u * 24, u * 90, u * 24, "НАЗАД", C_GRAY, u * 1.2f) || gBackPressed) { gBackPressed = false; gScreen = SC_MENU; }
}

// ---------------------------------------------------------------------
//  Меню, начало смены, пауза, итоги, памятка
// ---------------------------------------------------------------------
static float gMenuOrbit = 0;
static void drawMenuScene(float dt) {
    // фоновая сцена: патрульная машина у поста
    gMenuOrbit += dt * 0.08f;
    gWorldTime += dt;
    updateLighting(18.6f, 0);
    const Post& P = POSTS[0];
    PL.pos = P.pos; PL.yaw = P.yaw; PL.flash = true; PL.speed = 0; PL.spec = SV.car;
    V3 c = P.pos + V3(0, 0.8f, 0);
    V3 cam = c + V3(sinf(gMenuOrbit) * 9, 2.4f, cosf(gMenuOrbit) * 9);
    V3 d = c - cam;
    R3.setCamera(cam, atan2f(d.x, d.z), atan2f(d.y, sqrtf(d.x * d.x + d.z * d.z)), 60);
    if (CARS.empty()) for (int i = 0; i < 14; i++) spawnTrafficCar(P.pos, 20, 300, false);
    updateTraffic(dt, true);
    updatePeds(dt, P.pos, true);
    updateParts(dt);
    drawWorldScene(dt, true, false);
}
static void drawMenu(float dt) {
    float u = U();
    drawMenuScene(dt);
    UI.rectGH(0, 0, SW * 0.6f, SH, RGBA(0, 0, 0, 190), RGBA(0, 0, 0, 0));
    float mx = std::max(SW * 0.05f, 16.0f);
    UI.text(mx, SH * 0.1f, "ИНСПЕКТОР", u * 4.2f, C_WHITE, 0, true);
    UI.text(mx, SH * 0.1f + u * 34, "ДПС", u * 4.2f, RGBA(80, 140, 255), 0, true);
    UI.rect(mx, SH * 0.1f + u * 68, u * 150, u * 3, C_RED);
    UI.text(mx, SH * 0.1f + u * 76, fmt("СМЕНА №%d   %s", SV.day, dateStr(TODAY()).c_str()).c_str(), u * 1.1f, C_LIGHT, 0, true);
    UI.text(mx, SH * 0.1f + u * 88, fmt("%s   РЕПУТАЦИЯ %d   %d ₽", RANKS[rankOf(SV.rep)], SV.rep, SV.money).c_str(), u * 1.1f, C_YELLOW, 0, true);
    float bw = u * 150, bh = u * 25, bx = mx, by = SH * 0.48f;
    if (button(bx, by, bw, bh, "НАЧАТЬ СМЕНУ", C_BLUE, u * 1.3f)) {
        gScreen = SC_SHIFTSTART; gPlannedWeather = grng.chance(0.55f) ? 0 : (grng.chance(0.45f) ? 1 : 2); gPlannedNight = false;
        gPlannedDistrict = std::min(SV.lastDistrict, 3); if (rankOf(SV.rep) < DISTRICT_RANK[gPlannedDistrict]) gPlannedDistrict = 0;
    }
    by += bh + u * 6;
    if (button(bx, by, bw, bh, "ГАРАЖ", C_GRAY, u * 1.2f)) { gScreen = SC_GARAGE; gGarageSel = SV.car; }
    by += bh + u * 6;
    if (button(bx, by, bw, bh, "КАМЕРЫ НА КАРТЕ", C_GRAY, u * 1.2f)) { gMapReturn = SC_MENU; gScreen = SC_MAP; }
    by += bh + u * 6;
    if (button(bx, by, bw * 0.48f, bh, "ПАМЯТКА", C_GRAY, u * 1.0f)) { gHelpReturn = SC_MENU; gScreen = SC_HELP; }
    if (button(bx + bw * 0.52f, by, bw * 0.48f, bh, SV.sound ? "ЗВУК: ВКЛ" : "ЗВУК: ВЫКЛ", C_GRAY, u * 1.0f)) { SV.sound = !SV.sound; SY.muted = !SV.sound; saveGame(); }
    by += bh + u * 6;
    if (button(bx, by, bw, bh * 0.85f, "ВЫХОД", RGBA(110, 40, 40), u * 1.1f) || gBackPressed) { gBackPressed = false; gScreen = SC_EXITCONFIRM; }
    SY.engVol = 0.3f; SY.engRpm = 0.12f; SY.engLoad = 0; SY.siren = 0; SY.rain = 0; SY.ambient = 0.15f; SY.screech = 0;
}
static void drawExitConfirm(float dt) {
    float u = U();
    drawMenuScene(dt);
    UI.rect(0, 0, SW, SH, RGBA(0, 0, 0, 160));
    panel(SW / 2 - u * 100, SH / 2 - u * 40, u * 200, u * 80, C_DARK);
    UI.text(SW / 2, SH / 2 - u * 30, "ВЫЙТИ ИЗ ИГРЫ?", u * 1.4f, C_WHITE, 1);
    if (button(SW / 2 - u * 90, SH / 2 + u * 5, u * 85, u * 26, "ДА", C_RED, u * 1.2f)) { saveGame(); gQuit = true; }
    if (button(SW / 2 + u * 5, SH / 2 + u * 5, u * 85, u * 26, "НЕТ", C_GRAY, u * 1.2f) || gBackPressed) { gBackPressed = false; gScreen = SC_MENU; }
}
static void drawShiftStart(float dt) {
    float u = U();
    drawMenuScene(dt);
    UI.rect(0, 0, SW, SH, RGBA(0, 0, 0, 150));
    float mx = std::max(SW * 0.05f, 16.0f), mt = SH * 0.05f;
    float W = SW - 2 * mx;
    panel(mx, mt, W, SH - 2 * mt, C_DARK);
    UI.text(mx + u * 10, mt + u * 8, fmt("ЗАСТУПЛЕНИЕ НА СМЕНУ №%d  —  %s", SV.day, dateStr(TODAY()).c_str()).c_str(), u * 1.4f, C_WHITE);
    float y = mt + u * 28, cw = W * 0.48f;
    UI.text(mx + u * 10, y, "СМЕНА", u * 1.0f, RGBA(150, 200, 255)); y += u * 11;
    if (button(mx + u * 10, y, cw * 0.48f, u * 22, "ДНЕВНАЯ 08–16", !gPlannedNight ? C_BLUE : C_GRAY, u * 1.0f)) gPlannedNight = false;
    if (button(mx + u * 14 + cw * 0.48f, y, cw * 0.48f, u * 22, "НОЧНАЯ 20–04", gPlannedNight ? C_BLUE : C_GRAY, u * 1.0f)) gPlannedNight = true;
    y += u * 28;
    static const char* wnames[] = {"ЯСНО", "ПАСМУРНО", "ДОЖДЬ"};
    UI.text(mx + u * 10, y, (std::string("ПРОГНОЗ ПОГОДЫ: ") + wnames[gPlannedWeather]).c_str(), u * 1.1f, C_YELLOW); y += u * 16;
    UI.text(mx + u * 10, y, "ПАТРУЛЬНАЯ МАШИНА: ", u * 1.0f, RGBA(150, 200, 255));
    UI.text(mx + u * 10 + UI.textW("ПАТРУЛЬНАЯ МАШИНА: ", u), y, PATROL[SV.car].name, u * 1.0f, C_WHITE); y += u * 16;
    UI.text(mx + u * 10, y, "ЗАДАНИЯ ВЫДАЮТСЯ ПРИ ЗАСТУПЛЕНИИ.", u * 0.9f, C_LIGHT); y += u * 11;
    UI.text(mx + u * 10, y, "ДЛИТЕЛЬНОСТЬ СМЕНЫ ~12 МИНУТ.", u * 0.9f, C_LIGHT);
    // участки
    float rx = mx + cw + u * 20, ry = mt + u * 28, rw = W - cw - u * 30;
    UI.text(rx, ry, "УЧАСТОК ПАТРУЛИРОВАНИЯ", u * 1.0f, RGBA(150, 200, 255)); ry += u * 11;
    for (int d = 0; d < 4; d++) {
        bool ok = rankOf(SV.rep) >= DISTRICT_RANK[d];
        std::string l = ok ? DISTRICT_NAMES[d] : std::string(DISTRICT_NAMES[d]) + " (С ЗВАНИЯ " + RANKS[DISTRICT_RANK[d]] + ")";
        if (button(rx, ry, rw, u * 21, l.c_str(), gPlannedDistrict == d ? C_BLUE : C_GRAY, u * 0.95f, ok)) gPlannedDistrict = d;
        ry += u * 25;
    }
    float bh = u * 28;
    if (button(SW - mx - u * 170, SH - mt - bh - u * 8, u * 160, bh, "НА ЛИНИЮ!", C_GREEN, u * 1.4f)) startShift();
    if (button(mx + u * 10, SH - mt - bh - u * 8, u * 100, bh, "НАЗАД", C_GRAY, u * 1.2f) || gBackPressed) { gBackPressed = false; gScreen = SC_MENU; }
}
static void drawPause() {
    float u = U();
    UI.rect(0, 0, SW, SH, RGBA(0, 0, 0, 160));
    float w = u * 190, h = u * 190, x = SW / 2 - w / 2, y = SH / 2 - h / 2;
    panel(x, y, w, h, C_DARK);
    UI.text(SW / 2, y + u * 8, "ПАУЗА", u * 1.8f, C_WHITE, 1);
    float by = y + u * 32, bh = u * 24;
    if (button(x + u * 10, by, w - u * 20, bh, "ПРОДОЛЖИТЬ", C_BLUE, u * 1.2f) || gBackPressed) { gBackPressed = false; gScreen = SC_DRIVE; }
    by += bh + u * 6;
    if (button(x + u * 10, by, w - u * 20, bh, "КАРТА И КАМЕРЫ", C_GRAY, u * 1.1f)) { gMapReturn = SC_PAUSE; gScreen = SC_MAP; TOASTS.clear(); }
    by += bh + u * 6;
    if (button(x + u * 10, by, w - u * 20, bh, "ПАМЯТКА", C_GRAY, u * 1.1f)) { gHelpReturn = SC_PAUSE; gScreen = SC_HELP; }
    by += bh + u * 6;
    if (button(x + u * 10, by, (w - u * 24) / 2, bh, SV.sound ? "ЗВУК ВКЛ" : "ЗВУК ВЫКЛ", C_GRAY, u * 1.0f)) { SV.sound = !SV.sound; SY.muted = !SV.sound; }
    if (button(x + u * 14 + (w - u * 24) / 2, by, (w - u * 24) / 2, bh, "ЗАКОНЧИТЬ", RGBA(150, 50, 40), u * 1.0f)) { endShift(); }
    SY.engVol = 0; SY.siren = 0; SY.screech = 0;
}
static void drawSummary(float dt) {
    float u = U();
    drawMenuScene(dt);
    UI.rect(0, 0, SW, SH, RGBA(0, 0, 0, 160));
    float mx = std::max(SW * 0.08f, 16.0f), mt = SH * 0.06f;
    float W = SW - 2 * mx, H = SH - 2 * mt;
    panel(mx, mt, W, H, C_DARK);
    UI.text(SW / 2, mt + u * 8, "ИТОГИ СМЕНЫ", u * 1.8f, C_WHITE, 1);
    float y = mt + u * 32, x1 = mx + u * 14, x2 = mx + W * 0.52f;
    auto row = [&](float x, float& yy, const char* l, const std::string& v, SDL_Color c) { UI.text(x, yy, l, u * 1.0f, C_LIGHT); UI.text(x + W * 0.36f, yy, v.c_str(), u * 1.0f, c, 2); yy += u * 12; };
    float ya = y, yb = y;
    row(x1, ya, "ПРОВЕРЕНО МАШИН:", fmt("%d", SHF.checks), C_WHITE);
    row(x1, ya, "ВЕРНЫХ РЕШЕНИЙ:", fmt("%d", SHF.correct), RGBA(120, 255, 140));
    row(x1, ya, "ОШИБОК:", fmt("%d", SHF.wrong), SHF.wrong ? RGBA(255, 120, 100) : C_WHITE);
    row(x1, ya, "ШТРАФОВ:", fmt("%d НА %d ₽", SHF.fines, SHF.finesSum), C_WHITE);
    row(x1, ya, "ЗАДЕРЖАНО:", fmt("%d", SHF.detains), C_WHITE);
    row(x1, ya, "ПОГОНЬ / УСПЕШНЫХ:", fmt("%d / %d", SHF.chases, SHF.chaseWins), C_WHITE);
    row(x2, yb, "ЗАРПЛАТА:", fmt("%d ₽", (int)SHF.lastCamMin), C_YELLOW);
    row(x2, yb, "ПРЕМИИ И ЗАДАНИЯ:", fmt("%d ₽", SHF.bonus), C_YELLOW);
    row(x2, yb, "ДОХОД С КАМЕР:", fmt("%d ₽", SHF.camIncome), C_YELLOW);
    row(x2, yb, "ИЗМЕНЕНИЕ СЧЁТА:", fmt("%+d ₽", SHF.startMoney), SHF.startMoney >= 0 ? RGBA(120, 255, 140) : RGBA(255, 120, 100));
    row(x2, yb, "РЕПУТАЦИЯ:", fmt("%+d (ВСЕГО %d)", SHF.repDelta, SV.rep), C_WHITE);
    y = std::max(ya, yb) + u * 6;
    for (int i = 0; i < 3; i++) { UI.text(x1, y, (std::string(SHF.tasks[i].done ? "[X] " : "[ ] ") + taskText(SHF.tasks[i])).c_str(), u * 1.0f, SHF.tasks[i].done ? RGBA(120, 255, 140) : RGBA(170, 170, 180)); y += u * 11; }
    int rk = rankOf(SV.rep);
    if (rk > SHF.startRank) { y += u * 4; UI.text(SW / 2, y, (std::string("ПОВЫШЕНИЕ! ПРИСВОЕНО ЗВАНИЕ: ") + RANKS[rk]).c_str(), u * 1.3f, C_YELLOW, 1, true); }
    if (button(SW / 2 - u * 70, mt + H - u * 34, u * 140, u * 26, "В ГЛАВНОЕ МЕНЮ", C_BLUE, u * 1.2f) || gBackPressed) { gBackPressed = false; clearWorldActors(); gScreen = SC_MENU; }
}
static void drawHelp() {
    float u = U();
    UI.rect(0, 0, SW, SH, RGBA(12, 16, 24, 250));
    float mx = std::max(SW * 0.05f, 16.0f), mt = SH * 0.05f;
    UI.text(mx, mt, "ПАМЯТКА ИНСПЕКТОРА", u * 1.8f, C_WHITE);
    float colW = (SW - 2 * mx - u * 10) / 2;
    const char* left =
        "ЗАДЕРЖАНИЕ: ПОДДЕЛЬНОЕ УДОСТОВЕРЕНИЕ (ОШИБКА В ЗАГОЛОВКЕ, НЕТ ГОЛОГРАММЫ, НЕТ В БАЗЕ), НОМЕРА НЕ СОВПАДАЮТ СО СТС, АВТОМОБИЛЬ В УГОНЕ, КОНТРАБАНДА ИЛИ КРАДЕНОЕ, ПОПЫТКА СКРЫТЬСЯ.\n"
        "ЭВАКУАЦИЯ: НЕТ ДЕЙСТВУЮЩИХ ПРАВ — УДОСТОВЕРЕНИЕ ПРОСРОЧЕНО (П. 4В), ЧУЖОЕ (ДРУГОЕ ФОТО И ИМЯ), ЛИШЁН ПРАВ (ВИДНО В БАЗЕ).\n"
        "ШТРАФ: ПРЕВЫШЕНИЕ БОЛЕЕ 20 КМ/Ч (12.9), КРАСНЫЙ (12.12), НЕТ/ПРОСРОЧЕН ОСАГО, ОСАГО НА ДРУГУЮ МАШИНУ ИЛИ ВОДИТЕЛЬ НЕ ВПИСАН (12.37), НЕ ПРИСТЁГНУТ (12.6 — РЕМНЯ НЕТ НА ФОТО ВОДИТЕЛЯ). ОТМЕТЬ ВСЕ НАРУШЕННЫЕ СТАТЬИ.\n"
        "ПРЕДУПРЕЖДЕНИЕ: ПРЕВЫШЕНИЕ ДО 20 КМ/Ч БЕЗ ДРУГИХ НАРУШЕНИЙ.\n"
        "ОТПУСТИТЬ: ВСЁ В ПОРЯДКЕ. СОБСТВЕННИК В СТС МОЖЕТ БЫТЬ ДРУГИМ — ЭТО НЕ НАРУШЕНИЕ.";
    const char* right =
        "УПРАВЛЕНИЕ: СЛЕВА — РУЛЬ, СПРАВА — ГАЗ И ТОРМОЗ (ДОЛГО ДЕРЖАТЬ ТОРМОЗ = ЗАДНИЙ ХОД). МОЖНО ЖАТЬ ОДНОВРЕМЕННО.\n"
        "МИГАЛКА ВКЛЮЧАЕТ СВЕТ И СИРЕНУ — МАШИНЫ УСТУПАЮТ ДОРОГУ И ЧАЩЕ ПОДЧИНЯЮТСЯ.\n"
        "ТРЕБОВАТЬ (НА ХОДУ) ИЛИ ЖЕЗЛ (СТОЯ) — ОСТАНОВИТЬ МАШИНУ В ЖЁЛТОЙ РАМКЕ. РАДАР ПОКАЗЫВАЕТ СКОРОСТЬ МАШИНЫ ВПЕРЕДИ.\n"
        "ПОГОНЯ: ДЕРЖИСЬ БЛИЖЕ И ТОЛКАЙ БЕГЛЕЦА, ПОКА НЕ ЗАПОЛНИТСЯ ШКАЛА. ВЫЗОВИ ПОМОЩЬ И БЛОКПОСТ. ОТСТАНЕШЬ — ПРОВАЛ.\n"
        "ПРИДИРКИ К ЧЕСТНЫМ ВОДИТЕЛЯМ ВЕДУТ К ЖАЛОБАМ И ШТРАФАМ. ЗВАНИЯ ОТКРЫВАЮТ НОВЫЕ УЧАСТКИ И МАШИНЫ.";
    UI.textBox(mx, mt + u * 22, colW, left, u * 0.95f, C_LIGHT);
    UI.textBox(mx + colW + u * 10, mt + u * 22, colW, right, u * 0.95f, C_LIGHT);
    if (button(SW / 2 - u * 60, SH - mt - u * 26, u * 120, u * 24, "ПОНЯТНО", C_BLUE, u * 1.2f) || gBackPressed) { gBackPressed = false; gScreen = gHelpReturn; }
}

// ---------------------------------------------------------------------
//  Главный цикл
// ---------------------------------------------------------------------
static void handleEvent(const SDL_Event& e) {
    switch (e.type) {
        case SDL_QUIT: saveGame(); gQuit = true; break;
        case SDL_APP_WILLENTERBACKGROUND: case SDL_APP_TERMINATING: case SDL_APP_DIDENTERBACKGROUND:
            saveGame();
            if (gScreen == SC_DRIVE) gScreen = SC_PAUSE;
            if (gAudio) SDL_PauseAudioDevice(gAudio, 1);
            FINGERS.clear();
            gBackground = true;
            break;
        case SDL_APP_DIDENTERFOREGROUND: if (gAudio) SDL_PauseAudioDevice(gAudio, 0); FINGERS.clear(); gBackground = false; break;
        case SDL_RENDER_DEVICE_RESET:
            // контекст GPU потерян — пересоздаём атлас из копии в памяти
            if (atlasTex) SDL_DestroyTexture(atlasTex);
            if (atlasTexAdd) SDL_DestroyTexture(atlasTexAdd);
            createAtlasTextures();
            break;
        case SDL_WINDOWEVENT:
            if (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED || e.window.event == SDL_WINDOWEVENT_RESIZED) { int w, h; SDL_GetRendererOutputSize(gRen, &w, &h); SW = w; SH = h; DROPS.clear(); }
            if (e.window.event == SDL_WINDOWEVENT_FOCUS_LOST && gScreen == SC_DRIVE) { /* не ставим на паузу на ПК */ }
            break;
        case SDL_FINGERDOWN: {
            Finger f; f.id = e.tfinger.fingerId; f.x = e.tfinger.x * SW; f.y = e.tfinger.y * SH;
            FINGERS.push_back(f); TAPS.push_back({f.x, f.y, false});
        } break;
        case SDL_FINGERMOTION:
            for (Finger& f : FINGERS) if (f.id == e.tfinger.fingerId) { f.x = e.tfinger.x * SW; f.y = e.tfinger.y * SH; }
            break;
        case SDL_FINGERUP:
            for (size_t i = 0; i < FINGERS.size(); i++) if (FINGERS[i].id == e.tfinger.fingerId) { FINGERS.erase(FINGERS.begin() + i); break; }
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (e.button.which == SDL_TOUCH_MOUSEID) break;
            { Finger f; f.id = (SDL_FingerID)-777; f.x = (float)e.button.x * SW / std::max(1, SW); f.y = (float)e.button.y;
              int ww, wh; SDL_GetWindowSize(gWin, &ww, &wh); f.x = e.button.x * (float)SW / ww; f.y = e.button.y * (float)SH / wh;
              FINGERS.push_back(f); TAPS.push_back({f.x, f.y, false}); }
            break;
        case SDL_MOUSEMOTION:
            if (e.motion.which == SDL_TOUCH_MOUSEID) break;
            { int ww, wh; SDL_GetWindowSize(gWin, &ww, &wh); for (Finger& f : FINGERS) if (f.id == (SDL_FingerID)-777) { f.x = e.motion.x * (float)SW / ww; f.y = e.motion.y * (float)SH / wh; } }
            break;
        case SDL_MOUSEBUTTONUP:
            if (e.button.which == SDL_TOUCH_MOUSEID) break;
            for (size_t i = 0; i < FINGERS.size(); i++) if (FINGERS[i].id == (SDL_FingerID)-777) { FINGERS.erase(FINGERS.begin() + i); break; }
            break;
        case SDL_KEYDOWN:
            gKeys[e.key.keysym.scancode] = true;
            if (e.key.keysym.sym == SDLK_AC_BACK || e.key.keysym.sym == SDLK_ESCAPE) gBackPressed = true;
            if (gScreen == SC_DRIVE && !e.key.repeat) {
                if (e.key.keysym.sym == SDLK_f) { PL.flash = !PL.flash; PL.siren = PL.flash; }
                if (e.key.keysym.sym == SDLK_e) demandStop();
                if (e.key.keysym.sym == SDLK_c) { gCamMode = 1 - gCamMode; updateCamera(0, true); }
                if (e.key.keysym.sym == SDLK_h) sfx(SFX_HORN);
            }
            break;
        case SDL_KEYUP: gKeys[e.key.keysym.scancode] = false; break;
    }
}

static void frame(float dt) {
    gTime += dt;
    SDL_SetRenderDrawColor(gRen, 0, 0, 0, 255);
    SDL_RenderClear(gRen);
    switch (gScreen) {
        case SC_MENU: drawMenu(dt); break;
        case SC_EXITCONFIRM: drawExitConfirm(dt); break;
        case SC_SHIFTSTART: drawShiftStart(dt); break;
        case SC_GARAGE: drawGarage(dt); break;
        case SC_HELP: drawHelp(); break;
        case SC_MAP: drawMap(); for (size_t i = 0; i < TOASTS.size();) { TOASTS[i].t -= dt; if (TOASTS[i].t <= 0) TOASTS.erase(TOASTS.begin() + i); else i++; } break;
        case SC_SUMMARY: drawSummary(dt); break;
        case SC_DRIVE: {
            // клавиатура (ПК)
            updateDrive(dt);
            if (gScreen != SC_DRIVE) { updateCamera(dt, false); drawWorldScene(dt, true, gCamMode == 1); break; }
            updateCamera(dt, false);
            drawWorldScene(dt, true, gCamMode == 1);
            drawRain(dt, gCamMode == 1);
            if (gCamMode == 1) drawCockpitOverlay();
            drawDriveHUD();
            if (gKeys[SDL_SCANCODE_LEFT] || gKeys[SDL_SCANCODE_A]) gIn.steer = -1;
            if (gKeys[SDL_SCANCODE_RIGHT] || gKeys[SDL_SCANCODE_D]) gIn.steer = 1;
            if (gKeys[SDL_SCANCODE_UP] || gKeys[SDL_SCANCODE_W]) gIn.throttle = 1;
            if (gKeys[SDL_SCANCODE_DOWN] || gKeys[SDL_SCANCODE_S] || gKeys[SDL_SCANCODE_SPACE]) gIn.brake = 1;
            if (gKeys[SDL_SCANCODE_RETURN] && gInspect >= 0 && fabsf(PL.speed) < 2) { gScreen = SC_DOCS; PL.speed = 0; gKeys[SDL_SCANCODE_RETURN] = false; }
        } break;
        case SC_DOCS: case SC_DECIDE: case SC_RESULT: case SC_PAUSE: {
            SY.engLoad = 0; SY.screech = 0; SY.siren = 0;
            if (gScreen != SC_PAUSE && (gInspect < 0 || gInspect >= (int)CARS.size() || !CARS[gInspect].alive || CARS[gInspect].caseIdx < 0) && gScreen != SC_RESULT) { gScreen = SC_DRIVE; break; }
            updateCamera(0, false);
            drawWorldScene(dt, true, gCamMode == 1);
            if (gCamMode == 1) drawCockpitOverlay();
            if (gScreen == SC_DOCS) drawDocs(dt);
            else if (gScreen == SC_DECIDE) drawDecide();
            else if (gScreen == SC_RESULT) drawResult();
            else drawPause();
        } break;
        case SC_TRUNK:
            SY.siren = 0; SY.engLoad = 0;
            if (gInspect < 0 || !CARS[gInspect].alive || CARS[gInspect].caseIdx < 0) { gScreen = SC_DRIVE; break; }
            drawTrunkScreen(dt);
            break;
    }
    UI.flush();
    TAPS.clear();
    gBackPressed = false;
}

#ifdef GAISHNIK_TEST
static void testHook(int frameNo, float& dt);
#endif

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    SDL_SetHint(SDL_HINT_ORIENTATIONS, "LandscapeLeft LandscapeRight");
    SDL_SetHint(SDL_HINT_ANDROID_TRAP_BACK_BUTTON, "1");
    SDL_SetHint(SDL_HINT_TOUCH_MOUSE_EVENTS, "0");
    SDL_SetHint(SDL_HINT_MOUSE_TOUCH_EVENTS, "0");
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS) != 0) { SDL_Log("SDL_Init: %s", SDL_GetError()); return 1; }
#ifdef __ANDROID__
    gWin = SDL_CreateWindow("Инспектор ДПС", 0, 0, 0, 0, SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_ALLOW_HIGHDPI);
#else
    gWin = SDL_CreateWindow("Инспектор ДПС", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
#endif
    if (!gWin) { SDL_Log("window: %s", SDL_GetError()); return 1; }
    gRen = SDL_CreateRenderer(gWin, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!gRen) gRen = SDL_CreateRenderer(gWin, -1, SDL_RENDERER_SOFTWARE);
    if (!gRen) { SDL_Log("renderer: %s", SDL_GetError()); return 1; }
    SDL_GetRendererOutputSize(gRen, &SW, &SH);
    if (SW < SH) std::swap(SW, SH);
    SDL_GetRendererOutputSize(gRen, &SW, &SH);
    buildAtlas(); createAtlasTextures();
    loadAllModels();
    CAR_CULL = -1;
    buildRoads(); buildScenery();
    for (int i = 0; i < 4; i++) platePixels(i, i < 2 ? "А000АА" : "А0166", 66, i >= 2);
    // путь сохранения
#ifdef __ANDROID__
    const char* ip = SDL_AndroidGetInternalStoragePath();
    gSavePath = std::string(ip ? ip : ".") + "/gaishnik_save.txt";
#else
    char* bp = SDL_GetBasePath();
    gSavePath = std::string(bp ? bp : "./") + "gaishnik_save.txt";
    if (bp) SDL_free(bp);
#endif
    loadGame();
    initAudio();
    SY.muted = !SV.sound;
    gCamMode = SV.camView;
    playerSetCar();
    Uint64 prev = SDL_GetPerformanceCounter();
    int frameNo = 0;
    while (!gQuit) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) handleEvent(e);
        if (gBackground) {
            // в фоне не рисуем — ждём возвращения
            if (SDL_WaitEventTimeout(&e, 200)) handleEvent(e);
            prev = SDL_GetPerformanceCounter();
            continue;
        }
        Uint64 now = SDL_GetPerformanceCounter();
        float dt = (float)((now - prev) / (double)SDL_GetPerformanceFrequency());
        prev = now;
        if (dt > 0.05f) dt = 0.05f;
        if (dt < 0.001f) dt = 0.001f;
#ifdef GAISHNIK_TEST
        testHook(frameNo, dt);
#endif
        frame(dt);
        SDL_RenderPresent(gRen);
        frameNo++;
        (void)frameNo;
    }
    saveGame();
    if (gAudio) SDL_CloseAudioDevice(gAudio);
    SDL_DestroyRenderer(gRen); SDL_DestroyWindow(gWin);
    SDL_Quit();
    return 0;
}
