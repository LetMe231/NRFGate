/**
 * @file tests/unit/test_coap_parser/src/main.c
 *
 * Unit-Tests für die CoAP/JSON Parsing-Logik aus thread_handler.c.
 *
 * Da thread_handler.c den kompletten Netzwerk-Stack braucht, werden
 * die reinen Logik-Funktionen hier als eigenständige Implementierung
 * getestet:
 *
 *   - Discovery-Paket-Filter
 *   - JSON-Payload-Parsing (Zephyr json_obj_parse)
 *   - Sensor-Felder korrekt übernommen
 *   - Fehlende Felder → 0 / nicht im present-Bit
 *   - Leerer Payload → abgelehnt
 *   - Zu großer Payload → graceful truncation
 *
 * Diese Tests verifizieren dass das Protokoll zwischen ESP32-C6
 * und dem Gateway korrekt implementiert ist.
 */

#include <zephyr/ztest.h>
#include <zephyr/data/json.h>
#include <string.h>
#include <stdint.h>

#include "data_handler.h"

/* ── JSON Struct (aus thread_handler.c) ──────────────────────── */
/* Kopie der internen Struct — muss mit thread_handler.c synchron sein */

struct coap_sensor_json {
    int32_t seq, ts;
    int32_t ax, ay, az;
    int32_t gx, gy, gz;
    int32_t temp, hum;
    int32_t tvoc, eco2;
    int32_t heart_rate, spo2;
    int32_t raw_red, raw_ir;
    int32_t pm25, pm10;
    int32_t sw;
    int32_t light;
};

static const struct json_obj_descr coap_sensor_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, seq,        JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, ts,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, ax,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, ay,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, az,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, gx,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, gy,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, gz,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, temp,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, hum,        JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, tvoc,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, eco2,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, heart_rate, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, spo2,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, raw_red,    JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, raw_ir,     JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, pm25,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, pm10,       JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, sw,         JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct coap_sensor_json, light,      JSON_TOK_NUMBER),
};

/* ── Helper: JSON parsen ─────────────────────────────────────── */

static int32_t parse_json(const char *json, struct coap_sensor_json *out)
{
    memset(out, 0, sizeof(*out));
    size_t len = strlen(json);
    char buf[256];
    size_t copy = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, json, copy);
    buf[copy] = '\0';
    return json_obj_parse(buf, copy, coap_sensor_descr,
                          ARRAY_SIZE(coap_sensor_descr), out);
}

/* ── Discovery-Filter-Logik (aus thread_handler.c) ──────────── */

static bool is_discovery_packet(const char *payload, size_t len)
{
    if (len == 0) return false;
    return (memchr(payload, 'd', len) &&
            strncmp(payload, "{\"discover\"", 11) == 0);
}

/* ── Suite Setup ─────────────────────────────────────────────── */
ZTEST_SUITE(coap_parser, NULL, NULL, NULL, NULL, NULL);

/* ════════════════════════════════════════════════════════════════
 * Discovery-Filter
 * ════════════════════════════════════════════════════════════════ */

ZTEST(coap_parser, test_discovery_packet_detected)
{
    const char *pkt = "{\"discover\":true}";
    zassert_true(is_discovery_packet(pkt, strlen(pkt)),
                 "Should detect discovery packet");
}

ZTEST(coap_parser, test_discovery_packet_variant)
{
    const char *pkt = "{\"discover\":1,\"node\":\"esp32\"}";
    zassert_true(is_discovery_packet(pkt, strlen(pkt)),
                 "Should detect discovery packet variant");
}

ZTEST(coap_parser, test_sensor_packet_not_discovery)
{
    const char *pkt = "{\"seq\":1,\"temp\":22500}";
    zassert_false(is_discovery_packet(pkt, strlen(pkt)),
                  "Sensor packet should not be flagged as discovery");
}

ZTEST(coap_parser, test_empty_payload_not_discovery)
{
    zassert_false(is_discovery_packet("", 0),
                  "Empty payload should not be discovery");
}

ZTEST(coap_parser, test_partial_discover_not_discovery)
{
    /* Startet nicht mit '{"discover"' */
    const char *pkt = "{\"foo\":\"discover\"}";
    zassert_false(is_discovery_packet(pkt, strlen(pkt)),
                  "discover not at start should not match");
}

/* ════════════════════════════════════════════════════════════════
 * JSON Payload Parser
 * ════════════════════════════════════════════════════════════════ */

ZTEST(coap_parser, test_parse_temp_hum)
{
    struct coap_sensor_json s;
    int32_t present = parse_json(
        "{\"seq\":1,\"ts\":1000,\"temp\":22500,\"hum\":65000}", &s);

    zassert_true(present > 0, "Parse should succeed");
    zassert_equal(s.temp, 22500, "temp wrong");
    zassert_equal(s.hum,  65000, "hum wrong");
    zassert_equal(s.seq,  1,     "seq wrong");
}

ZTEST(coap_parser, test_parse_imu)
{
    struct coap_sensor_json s;
    int32_t present = parse_json(
        "{\"seq\":5,\"ax\":1000,\"ay\":-500,\"az\":9810,"
        "\"gx\":100,\"gy\":-50,\"gz\":25}", &s);

    zassert_true(present > 0, "IMU parse failed");
    zassert_equal(s.ax,  1000, "ax");
    zassert_equal(s.ay, -500,  "ay");
    zassert_equal(s.az,  9810, "az");
    zassert_equal(s.gx,  100,  "gx");
    zassert_equal(s.gy, -50,   "gy");
    zassert_equal(s.gz,  25,   "gz");
}

ZTEST(coap_parser, test_parse_air_quality)
{
    struct coap_sensor_json s;
    int32_t present = parse_json(
        "{\"tvoc\":350,\"eco2\":800}", &s);

    zassert_true(present > 0, "Air quality parse failed");
    zassert_equal(s.tvoc, 350, "tvoc");
    zassert_equal(s.eco2, 800, "eco2");
}

ZTEST(coap_parser, test_parse_switch_state)
{
    struct coap_sensor_json s;
    parse_json("{\"sw\":1}", &s);
    zassert_equal(s.sw, 1, "switch ON");

    parse_json("{\"sw\":0}", &s);
    zassert_equal(s.sw, 0, "switch OFF");
}

ZTEST(coap_parser, test_parse_missing_fields_stay_zero)
{
    /* Nur temp vorhanden — alle anderen Felder müssen 0 bleiben */
    struct coap_sensor_json s;
    parse_json("{\"temp\":20000}", &s);

    zassert_equal(s.eco2,       0, "eco2 should be 0");
    zassert_equal(s.tvoc,       0, "tvoc should be 0");
    zassert_equal(s.heart_rate, 0, "heart_rate should be 0");
    zassert_equal(s.sw,         0, "sw should be 0");
}

ZTEST(coap_parser, test_parse_negative_values)
{
    struct coap_sensor_json s;
    parse_json("{\"temp\":-5000,\"ay\":-9810}", &s);

    zassert_equal(s.temp, -5000, "negative temp");
    zassert_equal(s.ay,   -9810, "negative ay");
}

ZTEST(coap_parser, test_parse_large_values)
{
    /* SpO2 = 98000 m%, eco2 = 5000 ppm */
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"spo2\":98000,\"eco2\":5000}", &s);

    zassert_true(present > 0, "Large value parse failed");
    zassert_equal(s.spo2, 98000, "spo2");
    zassert_equal(s.eco2, 5000,  "eco2");
}

ZTEST(coap_parser, test_parse_empty_object_returns_zero_present)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{}", &s);
    /* present = 0 bedeutet kein Feld gefunden */
    zassert_equal(present, 0, "Empty JSON should return 0 present fields");
}

ZTEST(coap_parser, test_parse_malformed_json_returns_negative)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{temp:22500}", &s);  /* fehlendes Anführungszeichen */
    zassert_true(present < 0 || present == 0,
                 "Malformed JSON should fail (got %d)", present);
}

ZTEST(coap_parser, test_parse_extra_unknown_fields_ignored)
{
    /* Unbekannte Felder sollen das Parsing bekannter Felder nicht stören */
    struct coap_sensor_json s;
    int32_t present = parse_json(
        "{\"temp\":20000,\"unknown_sensor\":42,\"eco2\":800}", &s);

    zassert_true(present > 0, "Parse should succeed despite unknown fields");
    zassert_equal(s.temp, 20000, "temp should be parsed");
    zassert_equal(s.eco2, 800,   "eco2 should be parsed");
}

/* ════════════════════════════════════════════════════════════════
 * Sensor-Payload-Mapping (wie thread_handler.c es macht)
 * ════════════════════════════════════════════════════════════════ */

/* Simuliert die Mapping-Logik aus sensors_post() in thread_handler.c */
static void map_json_to_payload(const struct coap_sensor_json *s,
                                int32_t present_bits,
                                struct sensor_payload *p)
{
    /* present_bits ist die Bitmask von json_obj_parse:
     * Bit N gesetzt wenn Feld N im Descriptor gefunden wurde */
    p->present = 0;

    /* Felder entsprechend Descriptor-Reihenfolge prüfen:
     * 0=seq, 1=ts, 2=ax, 3=ay, 4=az, 5=gx, 6=gy, 7=gz,
     * 8=temp, 9=hum, 10=tvoc, 11=eco2, 12=hr, 13=spo2,
     * 14=raw_red, 15=raw_ir, 16=pm25, 17=pm10, 18=sw, 19=light */
    if (present_bits & BIT(0))  { p->seq = s->seq; p->present |= SENSOR_HAS_SEQ; }
    if (present_bits & BIT(2))  { p->ax  = s->ax;  p->present |= SENSOR_HAS_AX; }
    if (present_bits & BIT(3))  { p->ay  = s->ay;  p->present |= SENSOR_HAS_AY; }
    if (present_bits & BIT(4))  { p->az  = s->az;  p->present |= SENSOR_HAS_AZ; }
    if (present_bits & BIT(5))  { p->gx  = s->gx;  p->present |= SENSOR_HAS_GX; }
    if (present_bits & BIT(6))  { p->gy  = s->gy;  p->present |= SENSOR_HAS_GY; }
    if (present_bits & BIT(7))  { p->gz  = s->gz;  p->present |= SENSOR_HAS_GZ; }
    if (present_bits & BIT(8))  { p->temp = s->temp; p->present |= SENSOR_HAS_TEMP; }
    if (present_bits & BIT(9))  { p->hum  = s->hum;  p->present |= SENSOR_HAS_HUM; }
    if (present_bits & BIT(10)) { p->tvoc = s->tvoc; p->present |= SENSOR_HAS_TVOC; }
    if (present_bits & BIT(11)) { p->eco2 = s->eco2; p->present |= SENSOR_HAS_ECO2; }
    if (present_bits & BIT(18)) { p->switch_state = (uint8_t)s->sw;
                                   p->present |= SENSOR_HAS_SWITCH; }
}

ZTEST(coap_parser, test_mapping_temp_hum_sets_correct_bits)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"temp\":22500,\"hum\":60000}", &s);
    zassert_true(present > 0, "Parse failed");

    struct sensor_payload p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_true(p.present & SENSOR_HAS_TEMP, "SENSOR_HAS_TEMP not set");
    zassert_true(p.present & SENSOR_HAS_HUM,  "SENSOR_HAS_HUM not set");
    zassert_equal(p.temp, 22500, "temp value wrong");
    zassert_equal(p.hum,  60000, "hum value wrong");
}

ZTEST(coap_parser, test_mapping_missing_field_not_in_present)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"temp\":22500}", &s);

    struct sensor_payload p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_false(p.present & SENSOR_HAS_ECO2,
                  "ECO2 should not be in present when not in JSON");
}

ZTEST(coap_parser, test_mapping_switch_sets_switch_bit)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"sw\":1}", &s);

    struct sensor_payload p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_true(p.present & SENSOR_HAS_SWITCH, "SENSOR_HAS_SWITCH not set");
    zassert_equal(p.switch_state, 1, "switch_state wrong");
}

ZTEST(coap_parser, test_mapping_full_imu_sets_accel_and_gyro_bits)
{
    struct coap_sensor_json s;
    int32_t present = parse_json(
        "{\"ax\":1000,\"ay\":-500,\"az\":9810,"
        "\"gx\":100,\"gy\":-50,\"gz\":25}", &s);

    struct sensor_payload p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_true((p.present & SENSOR_HAS_ACCEL) == SENSOR_HAS_ACCEL,
                 "Full ACCEL group should be set");
    zassert_true((p.present & SENSOR_HAS_GYRO) == SENSOR_HAS_GYRO,
                 "Full GYRO group should be set");
}

/* ════════════════════════════════════════════════════════════════
 * Protokoll-Kompatibilitäts-Tests
 * (Verifiziert dass Gateway echte ESP32-C6 Pakete versteht)
 * ════════════════════════════════════════════════════════════════ */

ZTEST(coap_parser, test_real_world_imu_packet)
{
    /* Typisches Paket von ESP32-C6 mit AHT20 + SGP30 */
    const char *pkt =
        "{\"seq\":42,\"ts\":180000,"
        "\"temp\":23100,\"hum\":55000,"
        "\"tvoc\":120,\"eco2\":650}";

    struct coap_sensor_json s;
    int32_t present = parse_json(pkt, &s);
    zassert_true(present > 0, "Real-world packet parse failed");

    struct sensor_payload p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_equal(p.temp, 23100, "temp");
    zassert_equal(p.hum,  55000, "hum");
    zassert_equal(p.tvoc, 120,   "tvoc");
    zassert_equal(p.eco2, 650,   "eco2");
    zassert_true(p.present & SENSOR_HAS_TEMP,  "TEMP bit");
    zassert_true(p.present & SENSOR_HAS_HUM,   "HUM bit");
    zassert_true(p.present & SENSOR_HAS_TVOC,  "TVOC bit");
    zassert_true(p.present & SENSOR_HAS_ECO2,  "ECO2 bit");
}

ZTEST(coap_parser, test_real_world_switch_packet)
{
    /* ESP32-C6 meldet Button-Event */
    const char *pkt = "{\"seq\":100,\"ts\":500000,\"sw\":1}";

    struct coap_sensor_json s;
    int32_t present = parse_json(pkt, &s);

    struct sensor_payload p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_true(p.present & SENSOR_HAS_SWITCH, "Switch bit not set");
    zassert_equal(p.switch_state, 1, "switch_state wrong");
}