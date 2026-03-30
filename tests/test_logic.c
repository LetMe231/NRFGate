/**
 * @file test_logic.c
 * @brief Standalone unit tests for pure-logic functions in NRFGate.
 *
 * Compiles without the Zephyr SDK — Zephyr macros/types are mocked below.
 * Build & run:
 *   clang -Wall -Wextra -fsanitize=address,undefined -g \
 *         -I../inc -o test_logic test_logic.c && ./test_logic
 */

/* ── Zephyr stubs ────────────────────────────────────────────────── */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#define BIT(n)           (1u << (n))
#define ARRAY_SIZE(a)    (sizeof(a) / sizeof((a)[0]))
#define MIN(a, b)        ((a) < (b) ? (a) : (b))
#define MAX(a, b)        ((a) > (b) ? (a) : (b))

#define LOG_INF(fmt, ...)  ((void)0)
#define LOG_WRN(fmt, ...)  ((void)0)
#define LOG_ERR(fmt, ...)  ((void)0)
#define LOG_DBG(fmt, ...)  ((void)0)

/* Stubs for functions called by build_json */
typedef enum { NODE_STATE_UNKNOWN=0, NODE_STATE_IDLE, NODE_STATE_ACTIVE,
               NODE_STATE_ALERT, NODE_STATE_CRITICAL, NODE_STATE_LOST } node_state_t;
typedef enum { NODE_TRANSPORT_THREAD=0, NODE_TRANSPORT_BLE_MESH } node_transport_t;
#define NODE_ADDR_STR_LEN 46
#define MAX_NODES 8

static node_state_t _stub_state = NODE_STATE_IDLE;
static node_state_t semantic_handler_get_state(uint8_t idx) { (void)idx; return _stub_state; }
static const char  *semantic_handler_state_str(node_state_t s) {
    switch(s) {
        case NODE_STATE_IDLE:     return "IDLE";
        case NODE_STATE_ACTIVE:   return "ACTIVE";
        case NODE_STATE_ALERT:    return "ALERT";
        case NODE_STATE_CRITICAL: return "CRITICAL";
        case NODE_STATE_LOST:     return "LOST";
        default:                  return "UNKNOWN";
    }
}

/* ── Paste the structs from data_handler.h (minimal) ────────────── */
#define SENSOR_HAS_SEQ        BIT(0)
#define SENSOR_HAS_AX         BIT(2)
#define SENSOR_HAS_AY         BIT(3)
#define SENSOR_HAS_AZ         BIT(4)
#define SENSOR_HAS_GX         BIT(5)
#define SENSOR_HAS_GY         BIT(6)
#define SENSOR_HAS_GZ         BIT(7)
#define SENSOR_HAS_TEMP       BIT(8)
#define SENSOR_HAS_HUM        BIT(9)
#define SENSOR_HAS_TVOC       BIT(10)
#define SENSOR_HAS_ECO2       BIT(11)
#define SENSOR_HAS_HEART_RATE BIT(12)
#define SENSOR_HAS_SPO2       BIT(13)
#define SENSOR_HAS_RAW_RED    BIT(14)
#define SENSOR_HAS_RAW_IR     BIT(15)
#define SENSOR_HAS_ACCEL  (SENSOR_HAS_AX | SENSOR_HAS_AY | SENSOR_HAS_AZ)
#define SENSOR_HAS_GYRO   (SENSOR_HAS_GX | SENSOR_HAS_GY | SENSOR_HAS_GZ)

struct sensor_payload {
    int32_t present;
    int32_t seq;
    int32_t ts;
    int32_t ax, ay, az;
    int32_t gx, gy, gz;
    int32_t temp, hum;
    int32_t tvoc, eco2;
    int32_t heart_rate, spo2;
    int32_t raw_red, raw_ir;
};

typedef struct {
    node_transport_t transport;
    union {
        char     ipv6[NODE_ADDR_STR_LEN];
        uint16_t mesh_addr;
    };
} node_identity_t;

struct node_sensor_data {
    uint8_t          node_idx;
    node_identity_t  identity;
    int64_t          rx_uptime_ms;
    struct sensor_payload payload;
};

/* ── Copy the functions under test ──────────────────────────────── */

/* from model_handler.c */
static bool parse_mpid_format_a(uint16_t mpid, uint16_t *prop_id, uint8_t *data_len)
{
    if (mpid & 0x01) return false;
    *data_len = ((mpid >> 1) & 0x0F) + 1;
    *prop_id  = (mpid >> 5) & 0x7FF;
    return true;
}

static int32_t read_le_signed(const uint8_t *data, uint8_t len)
{
    switch (len) {
    case 1: return (int8_t)data[0];
    case 2: return (int16_t)(data[0] | (data[1] << 8));
    case 4: return (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    default: return 0;
    }
}

static uint32_t read_le_unsigned(const uint8_t *data, uint8_t len)
{
    switch (len) {
    case 1: return data[0];
    case 2: return (uint32_t)(data[0] | (data[1] << 8));
    case 4: return (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    default: return 0;
    }
}

/* from semantic_handler.c */
static int32_t mag2(int32_t ax, int32_t ay, int32_t az)
{
    int32_t x = ax/100, y = ay/100, z = az/100;
    return x*x + y*y + z*z;
}

static bool above_with_hyst(int32_t current, int32_t last, int32_t thresh, int32_t hyst)
{
    if (current >= thresh) return true;
    if (last > thresh && current > thresh - hyst) return true;
    return false;
}

/* from data_handler.c */
#define JSON_BUF_SIZE 220
#define WIRE_MAX_SIZE 48

static int build_json(const struct node_sensor_data *d, char *buf, size_t size)
{
    const struct sensor_payload *p = &d->payload;
    int off = 0;
    int rem;

#define JSON_APPEND(...) do {                             \
    rem = (int)size - off;                                \
    if (rem <= 0) { goto overflow; }                      \
    off += snprintf(buf + off, (size_t)rem, __VA_ARGS__); \
} while (0)

    const char *tr_tag = (d->identity.transport == NODE_TRANSPORT_BLE_MESH) ? "B" : "T";
    const char *state  = semantic_handler_state_str(semantic_handler_get_state(d->node_idx));
    JSON_APPEND("{\"node\":%d,\"tr\":\"%s\",\"state\":\"%s\"", d->node_idx, tr_tag, state);
    if (p->present & SENSOR_HAS_SEQ)  { JSON_APPEND(",\"seq\":%d", p->seq); }
    if (d->identity.transport == NODE_TRANSPORT_BLE_MESH) {
        JSON_APPEND(",\"mesh_addr\":%d", d->identity.mesh_addr);
    }
    if ((p->present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL) {
        JSON_APPEND(",\"ax\":%d.%03d,\"ay\":%d.%03d,\"az\":%d.%03d",
                    p->ax/1000, abs(p->ax%1000), p->ay/1000, abs(p->ay%1000),
                    p->az/1000, abs(p->az%1000));
    }
    if ((p->present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO) {
        JSON_APPEND(",\"gx\":%d.%03d,\"gy\":%d.%03d,\"gz\":%d.%03d",
                    p->gx/1000, abs(p->gx%1000), p->gy/1000, abs(p->gy%1000),
                    p->gz/1000, abs(p->gz%1000));
    }
    if (p->present & SENSOR_HAS_TEMP) {
        JSON_APPEND(",\"temp\":%d.%01d", p->temp/1000, abs((p->temp%1000)/100));
    }
    if (p->present & SENSOR_HAS_HUM)  {
        JSON_APPEND(",\"hum\":%d.%01d",  p->hum/1000,  abs((p->hum%1000)/100));
    }
    if (p->present & SENSOR_HAS_TVOC) { JSON_APPEND(",\"tvoc\":%d", p->tvoc); }
    if (p->present & SENSOR_HAS_ECO2) { JSON_APPEND(",\"eco2\":%d", p->eco2); }
    if (p->present & SENSOR_HAS_HEART_RATE) { JSON_APPEND(",\"hr\":%d", p->heart_rate); }
    if (p->present & SENSOR_HAS_SPO2) {
        JSON_APPEND(",\"spo2\":%d.%01d", p->spo2/1000, abs((p->spo2%1000)/100));
    }
    JSON_APPEND("}\n");
#undef JSON_APPEND
    return off;
overflow:
    return 0;
}

#define WRITE_I16(buf, off, val) do { int16_t _v=(int16_t)(val); (buf)[(off)++]=(uint8_t)(_v); (buf)[(off)++]=(uint8_t)(_v>>8); } while(0)
#define WRITE_U16(buf, off, val) do { uint16_t _v=(uint16_t)(val); (buf)[(off)++]=(uint8_t)(_v); (buf)[(off)++]=(uint8_t)(_v>>8); } while(0)

static int build_wire(const struct node_sensor_data *d, uint8_t *buf, size_t size)
{
    if (size < WIRE_MAX_SIZE) return 0;
    const struct sensor_payload *p = &d->payload;
    int off = 0;
    buf[off++] = d->node_idx;
    buf[off++] = (uint8_t)d->identity.transport;
    buf[off++] = (uint8_t)(p->present & 0xFF);
    buf[off++] = (uint8_t)((p->present >> 8) & 0xFF);
    if ((p->present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL) {
        WRITE_I16(buf, off, p->ax); WRITE_I16(buf, off, p->ay); WRITE_I16(buf, off, p->az);
    }
    if ((p->present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO) {
        WRITE_I16(buf, off, p->gx); WRITE_I16(buf, off, p->gy); WRITE_I16(buf, off, p->gz);
    }
    if (p->present & SENSOR_HAS_TEMP) { WRITE_I16(buf, off, p->temp/10); }
    if (p->present & SENSOR_HAS_HUM)  { WRITE_I16(buf, off, p->hum/10);  }
    if (p->present & SENSOR_HAS_TVOC) { WRITE_U16(buf, off, p->tvoc); }
    if (p->present & SENSOR_HAS_ECO2) { WRITE_U16(buf, off, p->eco2); }
    if (p->present & SENSOR_HAS_HEART_RATE) { buf[off++] = (uint8_t)p->heart_rate; }
    if (p->present & SENSOR_HAS_SPO2) { WRITE_U16(buf, off, p->spo2/10); }
    return off;
}
#undef WRITE_I16
#undef WRITE_U16

/* ── Test Framework ──────────────────────────────────────────────── */
static int pass_count = 0;
static int fail_count = 0;

#define CHECK(cond, msg) do {                                           \
    if (cond) {                                                         \
        printf("  PASS  %s\n", msg);                                    \
        pass_count++;                                                   \
    } else {                                                            \
        printf("  FAIL  %s  (line %d)\n", msg, __LINE__);              \
        fail_count++;                                                   \
    }                                                                   \
} while (0)

#define CHECK_EQ(a, b, msg) do {                                        \
    if ((a) == (b)) {                                                   \
        printf("  PASS  %s\n", msg);                                    \
        pass_count++;                                                   \
    } else {                                                            \
        printf("  FAIL  %s: got %lld, expected %lld  (line %d)\n",     \
               msg, (long long)(a), (long long)(b), __LINE__);         \
        fail_count++;                                                   \
    }                                                                   \
} while (0)

/* ── Tests ───────────────────────────────────────────────────────── */

static void test_parse_mpid(void)
{
    printf("\n[parse_mpid_format_a]\n");
    uint16_t prop; uint8_t dlen;

    /* Format B (bit0=1) must be rejected */
    CHECK(!parse_mpid_format_a(0x0001, &prop, &dlen), "Format B rejected");

    /* Temperature 0x004F, 2-byte payload
     * Format A: bit0=0, len_field=(2-1)=1 → bits[1..4]=0b0010
     * prop_id = 0x004F → bits[5..15] = 0x004F << 5 = 0x09E0
     * MPID = (0x004F << 5) | (1 << 1) | 0 = 0x09E2 */
    uint16_t mpid_temp = (0x004F << 5) | (1 << 1);
    CHECK(parse_mpid_format_a(mpid_temp, &prop, &dlen), "Temp MPID valid");
    CHECK_EQ(prop, 0x004F, "Temp prop_id");
    CHECK_EQ(dlen, 2,      "Temp data_len");

    /* 1-byte property (len_field=0) */
    uint16_t mpid_1b = (0x0100 << 5) | (0 << 1);
    CHECK(parse_mpid_format_a(mpid_1b, &prop, &dlen), "1-byte MPID valid");
    CHECK_EQ(dlen, 1, "1-byte data_len");

    /* max length field (0xF = 15 → data_len = 16) */
    uint16_t mpid_max = (0x001 << 5) | (0xF << 1);
    CHECK(parse_mpid_format_a(mpid_max, &prop, &dlen), "Max-len MPID valid");
    CHECK_EQ(dlen, 16, "Max data_len=16");
}

static void test_read_le(void)
{
    printf("\n[read_le_signed / read_le_unsigned]\n");

    /* Signed 1-byte */
    uint8_t neg1[] = {0xFF};
    CHECK_EQ(read_le_signed(neg1, 1), -1, "int8 -1");

    uint8_t pos42[] = {42};
    CHECK_EQ(read_le_signed(pos42, 1), 42, "int8 42");

    /* Signed 2-byte little-endian */
    uint8_t neg100[] = {0x9C, 0xFF};   /* -100 = 0xFF9C */
    CHECK_EQ(read_le_signed(neg100, 2), -100, "int16 -100");

    uint8_t pos1000[] = {0xE8, 0x03};  /* 1000 = 0x03E8 */
    CHECK_EQ(read_le_signed(pos1000, 2), 1000, "int16 1000");

    /* Signed 4-byte */
    uint8_t neg_big[] = {0x00, 0x00, 0x00, 0x80};  /* INT32_MIN */
    CHECK_EQ(read_le_signed(neg_big, 4), (int32_t)0x80000000, "int32 INT32_MIN");

    /* Unsigned 2-byte */
    uint8_t u16[] = {0xFF, 0xFF};
    CHECK_EQ(read_le_unsigned(u16, 2), 65535u, "uint16 65535");

    /* Unknown length returns 0 */
    CHECK_EQ(read_le_signed(pos42, 3), 0, "unknown len=3 returns 0");
    CHECK_EQ(read_le_unsigned(pos42, 3), 0u, "unsigned unknown len=3 returns 0");
}

static void test_above_with_hyst(void)
{
    printf("\n[above_with_hyst]\n");

    /* Clearly above threshold */
    CHECK(above_with_hyst(1100, 900, 1000, 100), "above thresh → true");

    /* Exactly at threshold */
    CHECK(above_with_hyst(1000, 900, 1000, 100), "at thresh → true");

    /* Below threshold, last was also below → false */
    CHECK(!above_with_hyst(800, 800, 1000, 100), "below, was below → false");

    /* Below threshold but within hyst band (last was above) */
    CHECK(above_with_hyst(950, 1050, 1000, 100), "in hyst band → true");

    /* Below hyst band (last was above) → should drop */
    CHECK(!above_with_hyst(890, 1050, 1000, 100), "below hyst band → false");

    /* Boundary: exactly at thresh-hyst (=900), last above */
    CHECK(!above_with_hyst(900, 1050, 1000, 100), "at thresh-hyst → false (not strictly >)");

    /* last == thresh exactly (boundary of hyst condition) */
    CHECK(!above_with_hyst(950, 1000, 1000, 100), "last==thresh → no hyst hold");
}

static void test_mag2(void)
{
    printf("\n[mag2]\n");

    /* Zero vector */
    CHECK_EQ(mag2(0, 0, 0), 0, "zero vector");

    /* 1g on Z axis: 1000 mg → x=y=0, z=10 → mag2=100 */
    CHECK_EQ(mag2(0, 0, 1000), 100, "1g on Z");

    /* Negative values same as positive (squared) */
    CHECK_EQ(mag2(0, 0, -1000), 100, "negative Z same as positive");

    /* Symmetric: (1000,1000,1000) → x=y=z=10 → 300 */
    CHECK_EQ(mag2(1000, 1000, 1000), 300, "symmetric 1g each axis");
}

static void test_build_json(void)
{
    printf("\n[build_json]\n");

    struct node_sensor_data d = {0};
    d.node_idx = 3;
    d.identity.transport = NODE_TRANSPORT_THREAD;
    d.payload.present = SENSOR_HAS_TEMP | SENSOR_HAS_HUM | SENSOR_HAS_SEQ;
    d.payload.seq  = 42;
    d.payload.temp = 22500;  /* 22.5 °C  (stored as m°C / 10 in wire, but /1000 in JSON) */
    d.payload.hum  = 60000;  /* 60.0 % */

    char buf[JSON_BUF_SIZE];
    int len = build_json(&d, buf, sizeof(buf));
    CHECK(len > 0, "build_json returns > 0");
    CHECK(buf[len-1] == '\n', "last char is newline");
    CHECK(strstr(buf, "\"node\":3") != NULL, "contains node id");
    CHECK(strstr(buf, "\"tr\":\"T\"")  != NULL, "transport tag Thread");
    CHECK(strstr(buf, "\"seq\":42")   != NULL, "seq present");
    CHECK(strstr(buf, "\"temp\":")    != NULL, "temp present");
    CHECK(strstr(buf, "\"hum\":")     != NULL, "hum present");
    CHECK(strstr(buf, "\"ax\":")     == NULL, "accel absent");
    printf("    JSON: %s", buf);

    /* Negative temperature: -5.5 °C = -5500 m°C */
    d.payload.temp = -5500;
    d.payload.present = SENSOR_HAS_TEMP;
    len = build_json(&d, buf, sizeof(buf));
    CHECK(len > 0, "negative temp: build_json ok");
    CHECK(strstr(buf, "\"-") == NULL, "negative temp: no malformed '\":-'");
    printf("    Negative temp JSON: %s", buf);

    /* BLE Mesh transport */
    d.identity.transport  = NODE_TRANSPORT_BLE_MESH;
    d.identity.mesh_addr  = 0x0005;
    d.payload.present     = SENSOR_HAS_HEART_RATE | SENSOR_HAS_SPO2;
    d.payload.heart_rate  = 72;
    d.payload.spo2        = 98000;  /* 98.0% */
    len = build_json(&d, buf, sizeof(buf));
    CHECK(len > 0, "BLE Mesh build_json ok");
    CHECK(strstr(buf, "\"tr\":\"B\"")       != NULL, "transport tag BLE");
    CHECK(strstr(buf, "\"mesh_addr\":5")    != NULL, "mesh_addr present");
    CHECK(strstr(buf, "\"hr\":72")          != NULL, "heart rate present");
    printf("    BLE JSON: %s", buf);

    /* Buffer too small → returns 0 (overflow guard) */
    char tiny[10];
    int r = build_json(&d, tiny, sizeof(tiny));
    CHECK_EQ(r, 0, "overflow: tiny buffer returns 0");
}

static void test_build_wire(void)
{
    printf("\n[build_wire]\n");

    struct node_sensor_data d = {0};
    d.node_idx = 2;
    d.identity.transport = NODE_TRANSPORT_THREAD;
    d.payload.present = SENSOR_HAS_ACCEL | SENSOR_HAS_GYRO |
                        SENSOR_HAS_TEMP   | SENSOR_HAS_HUM  |
                        SENSOR_HAS_TVOC   | SENSOR_HAS_ECO2;
    d.payload.ax = 100; d.payload.ay = -200; d.payload.az = 9810;
    d.payload.gx = 0;   d.payload.gy = 500;  d.payload.gz = -300;
    d.payload.temp = 22500; d.payload.hum = 55000;
    d.payload.tvoc = 120;   d.payload.eco2 = 850;

    uint8_t buf[WIRE_MAX_SIZE];
    int len = build_wire(&d, buf, sizeof(buf));
    CHECK(len > 0, "build_wire returns > 0");
    CHECK(len <= (int)WIRE_MAX_SIZE, "build_wire within WIRE_MAX_SIZE");

    /* Header bytes */
    CHECK_EQ(buf[0], 2,                      "node_idx in header");
    CHECK_EQ(buf[1], NODE_TRANSPORT_THREAD,  "transport in header");

    /* present bitmask little-endian */
    uint16_t present_wire = (uint16_t)(buf[2] | (buf[3] << 8));
    CHECK_EQ(present_wire, (uint16_t)(d.payload.present & 0xFFFF), "present bitmask correct");

    /* Round-trip temp: stored as temp/10 → 2250, read back little-endian */
    /* temp is at offset 4 + 6(accel) + 6(gyro) = 16 */
    int16_t temp_wire = (int16_t)(buf[16] | (buf[17] << 8));
    CHECK_EQ(temp_wire, 22500/10, "temp round-trip");

    /* Buffer too small */
    uint8_t small[3];
    CHECK_EQ(build_wire(&d, small, sizeof(small)), 0, "too-small buffer returns 0");

    /* Worst-case size stays within WIRE_MAX_SIZE */
    d.payload.present = SENSOR_HAS_ACCEL | SENSOR_HAS_GYRO |
                        SENSOR_HAS_TEMP  | SENSOR_HAS_HUM  |
                        SENSOR_HAS_TVOC  | SENSOR_HAS_ECO2 |
                        SENSOR_HAS_HEART_RATE | SENSOR_HAS_SPO2;
    d.payload.heart_rate = 75;
    d.payload.spo2 = 97000;
    len = build_wire(&d, buf, sizeof(buf));
    CHECK(len <= (int)WIRE_MAX_SIZE, "worst-case fits in WIRE_MAX_SIZE");
    printf("    Worst-case wire size: %d / %d bytes\n", len, WIRE_MAX_SIZE);
}

/* ── Main ────────────────────────────────────────────────────────── */
int main(void)
{
    printf("===== NRFGate Logic Tests =====\n");

    test_parse_mpid();
    test_read_le();
    test_above_with_hyst();
    test_mag2();
    test_build_json();
    test_build_wire();

    printf("\n===== Results: %d passed, %d failed =====\n",
           pass_count, fail_count);
    return fail_count > 0 ? 1 : 0;
}
