# Gateway Unit Tests

Zephyr Ztest-basierte Unit-Tests, laufen auf dem Host via `native_posix`.
Keine Hardware nötig.

## Voraussetzungen

- Zephyr SDK + west installiert
- `ZEPHYR_BASE` gesetzt (z.B. `export ZEPHYR_BASE=~/zephyrproject/zephyr`)

## Struktur

```
tests/unit/
  common/
    main.h              ← Stub für main.h (Scheduler-Typen)
  test_rule_engine/     ← rule_engine.c Tests (18 Tests)
  test_semantic/        ← semantic_handler.c Tests (16 Tests)
  test_data_handler/    ← data_handler.c Tests (18 Tests)
```

## Tests ausführen

```bash
# Einzelner Test-Build
cd tests/unit/test_rule_engine
west build -b native_posix .
west build -t run

# Alle Tests auf einmal (west twister)
cd <project_root>
west twister -T tests/unit --platform native_posix -v

# Mit Filter
west twister -T tests/unit --platform native_posix -k "rule_engine"
```

## Wichtige Hinweise

### identity_str() muss exportiert werden
`identity_str` in `data_handler.c` ist derzeit `static`-ähnlich (nur deklariert, 
nicht in Header). Für `test_data_handler` muss sie in `data_handler.h` exponiert werden:

```c
// data_handler.h hinzufügen:
void identity_str(const node_identity_t *id, char *buf, size_t len);
```

### data_handler_get_node_idx_by_ipv6 muss hinzugefügt werden
Der neue Lookup (für rule_engine.c nach dem fire()-Fix) muss deklariert sein:

```c
// data_handler.h:
uint8_t data_handler_get_node_idx_by_ipv6(const char *ipv6);
```

### build_json für JSON-Tests
Falls JSON-Builder-Tests gewünscht: `build_json` in `data_handler.c` von `static`
auf nicht-static ändern und in `data_handler.h` deklarieren, oder nur über
`data_handler_receive` + NUS-Capture testen (Blackbox-Ansatz).

## Test-Übersicht

### test_rule_engine (18 Tests)
| Test | Was wird geprüft |
|------|-----------------|
| test_add_returns_valid_index | add() gibt 0–15 zurück |
| test_add_max_rules_then_full | 17. Regel → -1 |
| test_remove_frees_slot | Slot wiederverwendbar |
| test_get_returns_null_for_empty_slot | Leerer Slot → NULL |
| test_fires_on_matching_state_and_node | Korrekte Regel feuert |
| test_no_fire_wrong_node | Falscher Node → kein Fire |
| test_no_fire_wrong_state | Falscher State → kein Fire |
| test_alert_trigger_matches_critical | ALERT feuert auch bei CRITICAL |
| test_critical_trigger_does_not_match_alert | CRITICAL feuert nicht bei ALERT |
| test_multiple_rules_same_node_both_fire | Beide Regeln feuern |
| test_switch_on_trigger_fires_on_sw_on | SWITCH_ON feuert bei sw=true |
| test_switch_on_trigger_does_not_fire_on_sw_off | SWITCH_ON nicht bei false |
| test_switch_off_trigger_fires_on_sw_off | SWITCH_OFF feuert bei sw=false |
| test_toggle_uses_target_state_not_source | TOGGLE: Ziel-State, nicht Quelle |
| test_toggle_unknown_state_defaults_to_on | Unbekannt → Toggle → ON |
| test_thread_target_calls_coap | CoAP PUT wird aufgerufen |
| test_actuator_cache_updated_on_target | Cache beim Target, nicht Source |
| test_json_* | JSON-Format korrekt |

### test_semantic (16 Tests)
| Test | Was wird geprüft |
|------|-----------------|
| test_first_packet_transitions_to_idle | UNKNOWN → IDLE |
| test_co2_normal_stays_idle | 400ppm → IDLE |
| test_co2_above_active_threshold | 1100ppm → ACTIVE (nach Sustain) |
| test_co2_above_alert_threshold | 2100ppm → ALERT |
| test_co2_above_critical_threshold | 5100ppm → CRITICAL |
| test_co2_hysteresis_stays_alert | Kurzer Rückgang → bleibt ALERT |
| test_co2_fast_rise_triggers_critical | +350ppm in 5s → CRITICAL |
| test_imu_high_motion_triggers_alert | Starke Bewegung → ALERT |
| test_node_lost_after_timeout | Kein Paket 30s → LOST |
| test_node_lost_notifies_nus | LOST sendet NUS-Nachricht |
| test_node_recovers_from_lost | Paket nach LOST → IDLE |
| test_transition_calls_rule_engine | Transition → rule_engine benachrichtigt |
| test_no_rule_call_without_transition | Gleichbleibender State → kein Aufruf |
| test_multiple_nodes_independent | Node 0 und 1 unabhängig |
| test_state_str_all_values | Alle State-Strings korrekt |

### test_data_handler (18 Tests)
| Test | Was wird geprüft |
|------|-----------------|
| test_first_thread_node_gets_index_0 | Erster Node → idx 0 |
| test_same_node_gets_same_index | Gleicher Node → gleicher idx |
| test_different_nodes_different_indices | Verschiedene Nodes → verschiedene idx |
| test_node_table_full_returns_0xff | Volle Tabelle → 0xFF |
| test_mesh_node_lookup_by_addr | BLE-Mesh-Lookup funktioniert |
| test_identity_str_thread | IPv6 korrekt zurückgegeben |
| test_identity_str_mesh | Mesh-Adresse in Hex |
| test_identity_str_no_shared_buffer | Kein statischer Buffer |
| test_cmd_* | NUS-Befehle korrekt geparst |
| test_cmd_garbage_no_crash | Garbage → kein Crash |