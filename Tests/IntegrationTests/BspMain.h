/**
 * \author Mr.Nobody
 * \file BspMain.h
 * \ingroup Pwr
 * \brief Entry point of Pwr integration test firmware (called by StartUp).
 *
 * StartUp calls BspMain() and gets this header through the interface library
 * BspMain_Lib (created by Tests/IntegrationTests/CMakeLists.txt). BspMain() is
 * implemented by ItTarget_Pwr.c.
 */

#ifndef PWR_ITTEST_BSPMAIN_H
#define PWR_ITTEST_BSPMAIN_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* ======================== EXPORTED FUNCTIONS ============================== */

void BspMain( void );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PWR_ITTEST_BSPMAIN_H */
