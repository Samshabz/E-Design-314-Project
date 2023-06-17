#ifndef INC_FUNC_H_
#define INC_FUNC_H_
void acknowledgeReset();
void writeMM();
void getProductNum();
void initTrackpad();
void azoInit();
void TP_ReATI();
void make_reset_certain();
void set_trackpad_settings();
void updateTrackpad();

uint8_t ReadTrackpadInfo();
void 	  HandleTrackpadEvent();


#endif
