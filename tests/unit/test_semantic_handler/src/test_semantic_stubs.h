/* test_semantic_stubs.h
 * Force-include wrapper for test_semantic_handler.
 * test_stubs.h sets #define SEMANTIC_HANDLER_H (guard mimic).
 * We undef it here so the real semantic_handler.h can be parsed
 * and semantic_thresholds_t becomes visible.
 */
#include "test_stubs.h"
#undef SEMANTIC_HANDLER_H
#include "semantic_handler.h"
