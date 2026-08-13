# vpd_predict
roof vent opener/closer with VPD prediction function

==========basic rules==========
1. #define must be written in uppercase letter.
   ex) #define VPD_PREDICT
   
2. first letter of each word in a function name must be uppercase.
   ex) void Vpd_Predict(void);
   
4. variable names must be written in lowercase letter(pointers are same too).
   ex) uint8_t vpd_predict;
       int *p;
   
5. array names must also be written in lowercase letters.
   ex) long long double vpd_predict[3];
   
6. name of typedef, struct, enum, union must be written in uppercase letter.
   ex) typedef struct {
         float vpd;
         float predict;
       }VPD_PREDICT;

8. variables shared between two or more .c files must be declared in the corresponding .h file using extern.
   ex)  In vpd_predict.h: extern uint8_t vpd_predict;
        In vpd_predict.c: uint8_t vpd_predict;

9. Constants shared between multiple .c files should be defined in the corresponding .h file.
    ex) #define MAX_SENSOR_COUNT 3

10. file-local variables and functions should use static.
    ex) static void Vpd_Predict(void);
        static void uint8_t vpd_predict;
