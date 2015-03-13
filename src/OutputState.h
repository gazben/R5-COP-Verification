#ifndef OutputState_h__
#define OutputState_h__

typedef enum OutputState{
  TRUE, FALSE, UNKNOWN
}OutputState;

/* 3 state logic functions */
OutputState AND_3(OutputState a, OutputState b){
  return (a == FALSE | b == FALSE) ? FALSE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : TRUE;
}
OutputState NAND_3(OutputState a, OutputState b){
  return (a == FALSE | b == FALSE) ? TRUE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : FALSE;
}
OutputState OR_3(OutputState a, OutputState b){
  return (a == TRUE | b == TRUE) ? TRUE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : FALSE;
}
OutputState XOR_3(OutputState a, OutputState b){
  return (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : (a == b) ? FALSE : TRUE;
}
OutputState NOT_3(OutputState a){
  return (a == UNKNOWN) ? UNKNOWN : (a == FALSE) ? TRUE : FALSE;
}
int isUnknown(OutputState a){
  return (a == UNKNOWN) ? 1 : 0;
}
#endif // OutputState_h__