/**
 * @file tests/unit/test_rule_engine/src/main.c
 * Fixed: no local enum redeclaration, ble_nus mock added
 */
#include <zephyr/ztest.h>
#include <string.h>
#include <stdbool.h>
#include "gw_model.h"
#include "rule_engine.h"

static int mock_cmd_count=0; static gw_cmd_type_t mock_last_cmd=GW_CMD_NONE;
static gw_node_addr_t mock_last_dst={0}; static int mock_store_idx=0;

int  command_router_send_to(const gw_node_addr_t *d,gw_cmd_type_t t)
     {mock_cmd_count++;mock_last_cmd=t;mock_last_dst=*d;return 0;}
int  command_router_send(const gw_command_t *c){ARG_UNUSED(c);return 0;}
int  gw_store_find_node(const gw_node_addr_t *a){ARG_UNUSED(a);return mock_store_idx;}
bool ble_nus_is_ready(void){return false;}
void ble_nus_send(const char *j){ARG_UNUSED(j);}

static void before_each(void *f){ARG_UNUSED(f);rule_engine_init();
  mock_cmd_count=0;mock_last_cmd=GW_CMD_NONE;memset(&mock_last_dst,0,sizeof(mock_last_dst));mock_store_idx=0;}
ZTEST_SUITE(rule_engine,NULL,NULL,before_each,NULL,NULL);

static int add_ble(uint8_t src,rule_trigger_t trig,rule_action_t act,uint16_t addr){
  gateway_rule_t r={.active=true,.src_node_idx=src,.trigger=trig,.action=act,.target_is_thread=false};
  r.target.mesh_addr=addr; return rule_engine_add(&r);}
static int add_thr(uint8_t src,rule_trigger_t trig,rule_action_t act,const char *ipv6){
  gateway_rule_t r={.active=true,.src_node_idx=src,.trigger=trig,.action=act,.target_is_thread=true};
  strncpy(r.target.ipv6,ipv6,GW_IPV6_STR_LEN-1); return rule_engine_add(&r);}
static gw_node_addr_t ba(uint16_t a){gw_node_addr_t x={.transport=GW_TR_BLE_MESH,.mesh_addr=a};return x;}

ZTEST(rule_engine,test_add_valid_index){int i=add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,0x0002);
  zassert_true(i>=0&&i<RULE_MAX,"");}
ZTEST(rule_engine,test_add_multiple_different){int i1=add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  int i2=add_ble(1,RULE_TRIG_STATE_ALERT,RULE_ACT_OFF,4);zassert_not_equal(i1,i2,"");}
ZTEST(rule_engine,test_get_returns_rule){int i=add_ble(2,RULE_TRIG_SWITCH_ON,RULE_ACT_TOGGLE,6);
  const gateway_rule_t *r=rule_engine_get((uint8_t)i);zassert_not_null(r,"");
  zassert_equal(r->src_node_idx,2,"");zassert_equal(r->trigger,RULE_TRIG_SWITCH_ON,"");
  zassert_equal(r->action,RULE_ACT_TOGGLE,"");zassert_equal(r->target.mesh_addr,6,"");}
ZTEST(rule_engine,test_get_invalid_null){zassert_is_null(rule_engine_get(RULE_MAX),"");
  zassert_is_null(rule_engine_get(255),"");}
ZTEST(rule_engine,test_get_empty_null){zassert_is_null(rule_engine_get(0),"");}
ZTEST(rule_engine,test_remove_makes_null){int i=add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  rule_engine_remove((uint8_t)i);zassert_is_null(rule_engine_get((uint8_t)i),"");}
ZTEST(rule_engine,test_remove_slot_reused){int i1=add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  rule_engine_remove((uint8_t)i1);int i2=add_ble(1,RULE_TRIG_STATE_IDLE,RULE_ACT_OFF,4);
  zassert_equal(i1,i2,"");}
ZTEST(rule_engine,test_remove_oob){rule_engine_remove(255);rule_engine_remove(RULE_MAX);zassert_true(true,"");}
ZTEST(rule_engine,test_table_full_error){for(int i=0;i<RULE_MAX;i++)add_ble((uint8_t)i,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  zassert_true(add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2)<0,"");}
ZTEST(rule_engine,test_add_null_error){zassert_true(rule_engine_add(NULL)<0,"");}

ZTEST(rule_engine,test_active_fires){add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;rule_engine_on_state(&s,GW_STATE_ACTIVE);
  zassert_equal(mock_cmd_count,1,"");zassert_equal(mock_last_cmd,GW_CMD_LIGHT_ON,"");}
ZTEST(rule_engine,test_wrong_state_no_fire){add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;rule_engine_on_state(&s,GW_STATE_IDLE);
  zassert_equal(mock_cmd_count,0,"");}
ZTEST(rule_engine,test_wrong_node_no_fire){add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  gw_node_addr_t s=ba(4);mock_store_idx=1;rule_engine_on_state(&s,GW_STATE_ACTIVE);
  zassert_equal(mock_cmd_count,0,"");}
ZTEST(rule_engine,test_alert_fires){add_ble(0,RULE_TRIG_STATE_ALERT,RULE_ACT_OFF,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;rule_engine_on_state(&s,GW_STATE_ALERT);
  zassert_equal(mock_cmd_count,1,"");zassert_equal(mock_last_cmd,GW_CMD_LIGHT_OFF,"");}
ZTEST(rule_engine,test_critical_triggers_alert_rule){add_ble(0,RULE_TRIG_STATE_ALERT,RULE_ACT_ON,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;rule_engine_on_state(&s,GW_STATE_CRITICAL);
  zassert_equal(mock_cmd_count,1,"escalation");}
ZTEST(rule_engine,test_toggle_action){add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_TOGGLE,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;rule_engine_on_state(&s,GW_STATE_ACTIVE);
  zassert_equal(mock_last_cmd,GW_CMD_LIGHT_TOGGLE,"");}
ZTEST(rule_engine,test_thread_target){add_thr(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,"fd11::1");
  gw_node_addr_t s=ba(4);mock_store_idx=0;rule_engine_on_state(&s,GW_STATE_ACTIVE);
  zassert_equal(mock_cmd_count,1,"");zassert_equal(mock_last_dst.transport,GW_TR_THREAD,"");}
ZTEST(rule_engine,test_multiple_rules_all_fire){add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_OFF,4);
  gw_node_addr_t s=ba(4);mock_store_idx=0;rule_engine_on_state(&s,GW_STATE_ACTIVE);
  zassert_equal(mock_cmd_count,2,"");}
ZTEST(rule_engine,test_null_src_no_crash){rule_engine_on_state(NULL,GW_STATE_ACTIVE);
  zassert_equal(mock_cmd_count,0,"");}
ZTEST(rule_engine,test_unknown_node_no_fire){add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  gw_node_addr_t s=ba(4);mock_store_idx=-1;rule_engine_on_state(&s,GW_STATE_ACTIVE);
  zassert_equal(mock_cmd_count,0,"");}

ZTEST(rule_engine,test_switch_on_fires){add_ble(0,RULE_TRIG_SWITCH_ON,RULE_ACT_TOGGLE,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;k_sleep(K_MSEC(1300));rule_engine_on_switch(&s,true);
  zassert_equal(mock_cmd_count,1,"");}
ZTEST(rule_engine,test_switch_off_fires){add_ble(0,RULE_TRIG_SWITCH_OFF,RULE_ACT_ON,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;k_sleep(K_MSEC(1300));rule_engine_on_switch(&s,false);
  zassert_equal(mock_cmd_count,1,"");}
ZTEST(rule_engine,test_switch_on_not_off_rule){add_ble(0,RULE_TRIG_SWITCH_OFF,RULE_ACT_ON,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;k_sleep(K_MSEC(1300));rule_engine_on_switch(&s,true);
  zassert_equal(mock_cmd_count,0,"");}
ZTEST(rule_engine,test_switch_cooldown){add_ble(0,RULE_TRIG_SWITCH_ON,RULE_ACT_ON,2);
  gw_node_addr_t s=ba(4);mock_store_idx=0;
  k_sleep(K_MSEC(1300));rule_engine_on_switch(&s,true);zassert_equal(mock_cmd_count,1,"");
  rule_engine_on_switch(&s,true);zassert_equal(mock_cmd_count,1,"cooldown");}
ZTEST(rule_engine,test_switch_null){rule_engine_on_switch(NULL,true);zassert_equal(mock_cmd_count,0,"");}

ZTEST(rule_engine,test_to_json_empty){char b[128];int l=rule_engine_to_json(b,sizeof(b));
  zassert_true(l>0,"");zassert_not_null(strstr(b,"rules"),"");zassert_not_null(strstr(b,"[]"),"");}
ZTEST(rule_engine,test_to_json_with_rule){add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  char b[256];zassert_true(rule_engine_to_json(b,sizeof(b))>0,"");}
ZTEST(rule_engine,test_to_json_null){zassert_equal(rule_engine_to_json(NULL,0),0,"");}
ZTEST(rule_engine,test_to_json_tiny){add_ble(0,RULE_TRIG_STATE_ACTIVE,RULE_ACT_ON,2);
  char b[8];rule_engine_to_json(b,sizeof(b));zassert_true(true,"");}
