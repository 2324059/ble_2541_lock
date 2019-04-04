#ifndef SIMPLEGATTPROFILE_H
#define SIMPLEGATTPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

// Profile Parameters
#define SIMPLEPROFILE_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value  3 value
#define SIMPLEPROFILE_CHAR4                   2  // RW uint8 - Profile Characteristic 4 value
  
#define GHOSTYU_UUID(uuid)     0x39, 0x23, 0xCF, 0x40,  0x73, 0x16, 0x42, 0x9A, 0x5C, 0x41, 0x7E, 0x7D, LO_UINT16(uuid), HI_UINT16(uuid), 0x83, 0x14
#define writeUUID(uuid)        0xA3, 0xE1, 0x26, 0x0A,  0xEE, 0x9A, 0xE9, 0xBB, 0xB0, 0x49, 0x0B, 0xEB, LO_UINT16(uuid), HI_UINT16(uuid), 0x00, 0x8B
#define notifyUUID(uuid)       0x92, 0x53, 0x19, 0xF2,  0x13, 0x5D, 0x9C, 0xB6, 0xBE, 0x43, 0x2B, 0x89, LO_UINT16(uuid), HI_UINT16(uuid), 0x04, 0xBA
// Simple Profile Service UUID
#define GHOSTYU_DEVICE_SERV_UUID              0xE207
#define SIMPLEPROFILE_SERV_UUID               0x9AC4 

// Key Pressed UUID
#define SIMPLEPROFILE_CHAR1_UUID            0xACE7//0xA3E1
#define SIMPLEPROFILE_CHAR4_UUID            0xC4B2//0x9253
  
// Simple Keys Profile Services bit fields
#define SIMPLEPROFILE_SERVICE               0x00000001

#define SIMPLEPROFILE_CHAR1_LEN           20
// Length of Characteristic 5 in bytes
#define SIMPLEPROFILE_CHAR5_LEN           5  
  
#if (defined USE_128_BIT_UUID)
#define UUID_SIZE 16
#else
#define UUID_SIZE 2
#endif
  
/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*simpleProfileChange_t)( uint8 paramID );

typedef struct
{
  simpleProfileChange_t        pfnSimpleProfileChange;  // Called when characteristic value changes
} simpleProfileCBs_t;

extern bStatus_t SimpleProfile_AddService( uint32 services );


extern bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks );

extern bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value );

extern bStatus_t SimpleProfile_GetParameter( uint8 param, void *value );

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEGATTPROFILE_H */
