/**
 * @file tests/unit/test_coap_parser/src/main.c
 *
 * Unit-Tests für die CoAP/JSON Parsing-Logik aus thread_adapter.c.
 */

#include <zephyr/ztest.h>
#include <zephyr/data/json.h>
#include <string.h>
#include <stdint.h>

#include "gw_model.h"

/* ── JSON Struct (identisch mit thread_adapter.c intern) ─────── */

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

/* ── Discovery-Filter ────────────────────────────────────────── */

static bool is_discovery_packet(const char *payload, size_t len)
{
    if (len == 0) return false;
    return (memchr(payload, 'd', len) &&
            strncmp(payload, "{\"discover\"", 11) == 0);
}

/* ── Mapping JSON → gw_sensor_payload_t ─────────────────────── */

static void map_json_to_payload(const struct coap_sensor_json *s,
                                int32_t present_bits,
                                gw_sensor_payload_t *p)
{
    /* Descriptor-Reihenfolge:
     * 0=seq, 1=ts, 2=ax, 3=ay, 4=az, 5=gx, 6=gy, 7=gz,
     * 8=temp, 9=hum, 10=tvoc, 11=eco2, 12=hr, 13=spo2,
     * 14=raw_red, 15=raw_ir, 16=pm25, 17=pm10, 18=sw, 19=light */
    p->present = 0;

    if (present_bits & BIT(0)) { p->seq      = s->seq; p->present |= GW_HAS_SEQ; }
    if (present_bits & BIT(2)) { p->ax_mg    = s->ax;  p->present |= GW_HAS_ACCEL; }
    if (present_bits & BIT(3)) { p->ay_mg    = s->ay; }
    if (present_bits & BIT(4)) { p->az_mg    = s->az; }
    if (present_bits & BIT(5)) { p->gx_mdps  = s->gx;  p->present |= GW_HAS_GYRO; }
    if (present_bits & BIT(6)) { p->gy_mdps  = s->gy; }
    if (present_bits & BIT(7)) { p->gz_mdps  = s->gz; }

    if (present_bits & BIT(8)) {
        p->temp_mc = s->temp;
        p->present |= GW_HAS_TEMP;
    }
    if (present_bits & BIT(9)) {
        p->hum_mpermille = s->hum;
        p->present |= GW_HAS_HUM;
    }
    if (present_bits & BIT(10)) {
        p->tvoc_ppb = s->tvoc;
        p->present |= GW_HAS_TVOC;
    }
    if (present_bits & BIT(11)) {
        p->eco2_ppm = s->eco2;
        p->present |= GW_HAS_ECO2;
    }
    if (present_bits & BIT(12)) {
        p->heart_rate_bpm = s->heart_rate;
        p->present |= GW_HAS_HEART_RATE;
    }
    if (present_bits & BIT(13)) {
        p->spo2_mpermille = s->spo2;
        p->present |= GW_HAS_SPO2;
    }
    if (present_bits & BIT(16)) {
        p->pm25_ugm3 = s->pm25;
        p->present |= GW_HAS_PM25;
    }
    if (present_bits & BIT(17)) {
        p->pm10_ugm3 = s->pm10;
        p->present |= GW_HAS_PM10;
    }
    if (present_bits & BIT(18)) {
        p->switch_state = (s->sw != 0);
        p->present |= GW_HAS_SWITCH;
    }
    if (present_bits & BIT(19)) {
        p->light_on = (s->light != 0);
        p->present |= GW_HAS_LIGHT;
    }
}

/* ── Suite ────────────────────────────────────────────────────── */
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
                  "Sensor packet should not be discovery");
}

ZTEST(coap_parser, test_empty_payload_not_discovery)
{
    zassert_false(is_discovery_packet("", 0),
                  "Empty should not be discovery");
}

ZTEST(coap_parser, test_partial_discover_not_discovery)
{
    const char *pkt = "{\"foo\":\"discover\"}";
    zassert_false(is_discovery_packet(pkt, strlen(pkt)),
                  "discover not at start should not match");
}

/* ════════════════════════════════════════════════════════════════
 * JSON Parser
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

ZTEST(coap_parser, test_parse_air_quality)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"tvoc\":350,\"eco2\":800}", &s);

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
    struct coap_sensor_json s;
    parse_json("{\"temp\":20000}", &s);

    zassert_equal(s.eco2,       0, "eco2 should be 0");
    zassert_equal(s.tvoc,       0, "tvoc should be 0");
    zassert_equal(s.heart_rate, 0, "hr should be 0");
    zassert_equal(s.sw,         0, "sw should be 0");
}

ZTEST(coap_parser, test_parse_negative_values)
{
    struct coap_sensor_json s;
    parse_json("{\"temp\":-5000,\"ay\":-9810}", &s);

    zassert_equal(s.temp, -5000, "negative temp");
    zassert_equal(s.ay,   -9810, "negative ay");
}

ZTEST(coap_parser, test_parse_empty_object_returns_zero)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{}", &s);
    zassert_equal(present, 0, "Empty JSON should return 0 present");
}

ZTEST(coap_parser, test_parse_malformed_fails)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{temp:22500}", &s);
    zassert_true(present <= 0,
                 "Malformed JSON should fail (got %d)", present);
}

ZTEST(coap_parser, test_parse_unknown_fields_ignored)
{
    struct coap_sensor_json s;
    int32_t present = parse_json(
        "{\"temp\":20000,\"unknown_sensor\":42,\"eco2\":800}", &s);

    zassert_true(present > 0, "Should succeed despite unknown fields");
    zassert_equal(s.temp, 20000, "temp should be parsed");
    zassert_equal(s.eco2, 800,   "eco2 should be parsed");
}

/* ════════════════════════════════════════════════════════════════
 * Mapping JSON → gw_sensor_payload_t
 * ════════════════════════════════════════════════════════════════ */

ZTEST(coap_parser, test_mapping_temp_hum_sets_correct_bits)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"temp\":22500,\"hum\":60000}", &s);

    gw_sensor_payload_t p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_true(p.present & GW_HAS_TEMP, "GW_HAS_TEMP not set");
    zassert_true(p.present & GW_HAS_HUM,  "GW_HAS_HUM not set");
    zassert_equal(p.temp_mc,       22500, "temp_mc wrong");
    zassert_equal(p.hum_mpermille, 60000, "hum_mpermille wrong");
}

ZTEST(coap_parser, test_mapping_missing_field_not_in_present)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"temp\":22500}", &s);

    gw_sensor_payload_t p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_false(p.present & GW_HAS_ECO2,
                  "ECO2 must not be in present when not in JSON");
}

ZTEST(coap_parser, test_mapping_switch_sets_switch_bit)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"sw\":1}", &s);

    gw_sensor_payload_t p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_true(p.present & GW_HAS_SWITCH, "GW_HAS_SWITCH not set");
    zassert_true(p.switch_state,            "switch_state wrong");
}

ZTEST(coap_parser, test_mapping_air_quality)
{
    struct coap_sensor_json s;
    int32_t present = parse_json("{\"tvoc\":350,\"eco2\":800}", &s);

    gw_sensor_payload_t p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_true(p.present & GW_HAS_TVOC, "GW_HAS_TVOC not set");
    zassert_true(p.present & GW_HAS_ECO2, "GW_HAS_ECO2 not set");
    zassert_equal(p.tvoc_ppb, 350, "tvoc_ppb wrong");
    zassert_equal(p.eco2_ppm, 800, "eco2_ppm wrong");
}

/* ════════════════════════════════════════════════════════════════
 * Protokoll-Kompatibilitäts-Tests
 * ════════════════════════════════════════════════════════════════ */

ZTEST(coap_parser, test_real_world_env_packet)
{
    const char *pkt =
        "{\"seq\":42,\"ts\":180000,"
        "\"temp\":23100,\"hum\":55000,"
        "\"tvoc\":120,\"eco2\":650}";

    struct coap_sensor_json s;
    int32_t present = parse_json(pkt, &s);
    zassert_true(present > 0, "Real-world packet parse failed");

    gw_sensor_payload_t p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_equal(p.temp_mc,       23100, "temp_mc wrong");
    zassert_equal(p.hum_mpermille, 55000, "hum_mpermille wrong");
    zassert_equal(p.tvoc_ppb,      120,   "tvoc_ppb wrong");
    zassert_equal(p.eco2_ppm,      650,   "eco2_ppm wrong");
    zassert_true(p.present & GW_HAS_TEMP, "TEMP bit");
    zassert_true(p.present & GW_HAS_HUM,  "HUM bit");
    zassert_true(p.present & GW_HAS_TVOC, "TVOC bit");
    zassert_true(p.present & GW_HAS_ECO2, "ECO2 bit");
}

ZTEST(coap_parser, test_real_world_switch_packet)
{
    const char *pkt = "{\"seq\":100,\"ts\":500000,\"sw\":1}";

    struct coap_sensor_json s;
    int32_t present = parse_json(pkt, &s);

    gw_sensor_payload_t p = {0};
    map_json_to_payload(&s, present, &p);

    zassert_true(p.present & GW_HAS_SWITCH, "GW_HAS_SWITCH not set");
    zassert_true(p.switch_state,            "switch_state wrong");
}
