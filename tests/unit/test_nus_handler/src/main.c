#include <zephyr/ztest.h>
#include <string.h>
#include <stdbool.h>
#include "gw_model.h"
#include "nus_handler.h"

static uint16_t    mock_unprov_addr=0;
static bool        mock_prov_started=false;
static bool        mock_mesh_reset_called=false;
static bool        mock_lora_enabled=false;
static bool        mock_lora_set_called=false;
static char        mock_nus_sent[512]={0};
static bool        mock_nus_ready_val=true;
static int         mock_rule_add_calls=0;
static gateway_rule_t mock_last_rule={0};
static int         mock_rule_rm_idx=-1;

static void reset_mocks(void){
    mock_unprov_addr=0;mock_prov_started=false;mock_mesh_reset_called=false;
    mock_lora_enabled=false;mock_lora_set_called=false;
    memset(mock_nus_sent,0,sizeof(mock_nus_sent));mock_nus_ready_val=true;
    mock_rule_add_calls=0;memset(&mock_last_rule,0,sizeof(mock_last_rule));
    mock_rule_rm_idx=-1;
}
void ble_mesh_prov_start_window(void)            { mock_prov_started=true; }
void ble_mesh_prov_unprovision_node(uint16_t a)  { mock_unprov_addr=a; }
void ble_mesh_prov_reconfigure_node(uint16_t a)  { ARG_UNUSED(a); }
void ble_mesh_prov_full_reset(void)              { mock_mesh_reset_called=true; }
void ble_mesh_prov_purge_lost_nodes(void)        { }
int  rule_engine_add(const gateway_rule_t *r)    { mock_last_rule=*r;return mock_rule_add_calls++; }
void rule_engine_remove(uint8_t i)               { mock_rule_rm_idx=(int)i; }
int  rule_engine_to_json(char *b,size_t s)       { return snprintf(b,s,"{\"rules\":[]}\n"); }
void lorawan_adapter_set_enabled(bool e)         { mock_lora_enabled=e;mock_lora_set_called=true; }
bool ble_nus_is_ready(void)                      { return mock_nus_ready_val; }
void ble_nus_send(const char *j)                 { strncpy(mock_nus_sent,j,sizeof(mock_nus_sent)-1); }
int  gw_store_find_node(const gw_node_addr_t *a) { ARG_UNUSED(a);return -1; }
int  command_router_send_to(const gw_node_addr_t *d,gw_cmd_type_t t){ARG_UNUSED(d);ARG_UNUSED(t);return 0;}
int  command_router_send(const gw_command_t *c)  { ARG_UNUSED(c);return 0; }
void mesh_scheduler_set_timing(uint32_t t,uint32_t b){ARG_UNUSED(t);ARG_UNUSED(b);}

static void before_each(void *f){ARG_UNUSED(f);reset_mocks();}
ZTEST_SUITE(nus_handler,NULL,NULL,before_each,NULL,NULL);
static void cmd(const char *j){nus_handler_cmd(j,strlen(j));}

ZTEST(nus_handler,test_start_prov)
    {cmd("{\"cmd\":\"start_prov\"}");zassert_true(mock_prov_started,"");}
ZTEST(nus_handler,test_start_prov_spaces)
    {cmd("{\"cmd\": \"start_prov\"}");zassert_true(mock_prov_started,"");}
ZTEST(nus_handler,test_unprovision_valid)
    {cmd("{\"cmd\":\"unprovision\",\"mesh_addr\":2}");zassert_equal(mock_unprov_addr,2,"");}
ZTEST(nus_handler,test_unprovision_high)
    {cmd("{\"cmd\":\"unprovision\",\"mesh_addr\":255}");zassert_equal(mock_unprov_addr,255,"");}
ZTEST(nus_handler,test_unprovision_no_addr)
    {cmd("{\"cmd\":\"unprovision\"}");zassert_equal(mock_unprov_addr,0,"");}
ZTEST(nus_handler,test_reset_mesh)
    {cmd("{\"cmd\":\"reset_mesh\"}");zassert_true(mock_mesh_reset_called,"");}
ZTEST(nus_handler,test_list_rules){
    cmd("{\"cmd\":\"list_rules\"}");
    zassert_true(strlen(mock_nus_sent)>0,"");
    zassert_not_null(strstr(mock_nus_sent,"rules"),"");
}
ZTEST(nus_handler,test_set_lora_false)
    {cmd("{\"cmd\":\"set_lora\",\"enabled\":false}");zassert_true(mock_lora_set_called,"");}
ZTEST(nus_handler,test_set_lora_no_crash)
    {cmd("{\"cmd\":\"set_lora\",\"enabled\":true}");zassert_true(true,"");}
ZTEST(nus_handler,test_unknown_cmd)
    {cmd("{\"cmd\":\"does_not_exist\"}");zassert_true(true,"");}
ZTEST(nus_handler,test_garbage){
    static const char *in[]={"","{}","{{{","AAA","{\"cmd\":}","null","[]"};
    for(int i=0;i<ARRAY_SIZE(in);i++) nus_handler_cmd(in[i],strlen(in[i]));
    zassert_true(true,"");
}
ZTEST(nus_handler,test_long_input){
    char b[512];memset(b,'A',sizeof(b)-1);b[sizeof(b)-1]=0;
    nus_handler_cmd(b,sizeof(b)-1);zassert_true(true,"");
}
