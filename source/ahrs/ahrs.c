#include "ext.h" // standard Max include, always required
#include "ext_obex.h" // required for new style Max object
#include <Fusion.h>

typedef struct _ahrs
{
    t_object ob;
    //t_clock *clock;
    FusionAhrs ahrs;
    FusionOffset offset;
    double gain;
    double accelRejection;
    double dt;
    long prevMs;
    
    FusionVector gyro;
    FusionVector accel;
    
    FusionVector gyroOffset;
    FusionVector accelOffset;
    
    bool callibrating;
    int callibration_samp;
    int target_samps;
    
    void *out;
    void *out2;
    void *out3;
    void *dump;
} t_ahrs;

//prototype
void* ahrs_new(t_symbol* s, long argc, t_atom* argv);
void ahrs_free(t_ahrs* x);
void ahrs_assist(t_ahrs* x, void* b, long m, long a, char* s);
void ahrs_list(t_ahrs *x, t_symbol *s, long argc, t_atom *argv);
void ahrs_anything(t_ahrs* x, t_symbol* s, long ac, t_atom* av);
void ahrs_update(t_ahrs *x);
void ahrs_callibrate(t_ahrs *x);

t_max_err ahrs_gain_set(t_ahrs* x, void* attr, long ac, t_atom* av);
t_max_err ahrs_accelRejection_set(t_ahrs* x, void* attr, long ac, t_atom* av);

void ahrs_update_settings(t_ahrs* x);

//global class pointser
void* ahrs_class;

void ext_main(void* r)
{
    t_class* c;

    c = class_new("ahrs", (method)ahrs_new, (method)ahrs_free, (long)sizeof(t_ahrs),
                  0L, A_GIMME, 0);

    class_addmethod(c, (method)ahrs_assist, "assist", A_CANT, 0);
    
    class_addmethod(c, (method)ahrs_list, "list", A_GIMME, 0);
    class_addmethod(c, (method)ahrs_anything, "anything", A_GIMME, 0);
    
    CLASS_ATTR_DOUBLE(c, "gain", 0, t_ahrs, gain);
    CLASS_ATTR_ACCESSORS(c, "gain", NULL, (method)ahrs_gain_set)
    CLASS_ATTR_LABEL(c, "gain", 0, "Gain");
    
    CLASS_ATTR_DOUBLE(c, "acceleration_rejection", 0, t_ahrs, accelRejection);
    CLASS_ATTR_ACCESSORS(c, "acceleration_rejection", NULL, (method)ahrs_accelRejection_set)
    CLASS_ATTR_LABEL(c, "acceleration_rejection", 0, "Acceleration rejection");
    
    CLASS_ATTR_FLOAT_ARRAY(c, "gyroscope_offset", 0, t_ahrs, gyroOffset, 3);
    CLASS_ATTR_LABEL(c, "gyroscope_offset", 0, "Gyroscope offset");
    
    CLASS_ATTR_FLOAT_ARRAY(c, "accelerometer_offset", 0, t_ahrs, accelOffset.array, 3);
    CLASS_ATTR_LABEL(c, "accelerometer_offset", 0, "Accelerometer offset");
    
    class_register(CLASS_BOX, c);
    ahrs_class = c;
}

void ahrs_assist(t_ahrs* x, void* b, long m, long a, char* s)
{
    if (m == ASSIST_INLET) { // inlet
        sprintf(s, "list of IMU data: gyroscope, acceleration, delta time (optional)");
    }
    else { // outlet
        switch (a) {
            case 0:
                sprintf(s, "orientation");
                break;
            case 1:
                sprintf(s, "acceleration");
                break;
            case 2:
                sprintf(s, "gravity");
                break;
            default:
                sprintf(s, "dumpout");
                break;
        }
        //sprintf(s, "I am outlet %ld", a);
    }
}

void ahrs_free(t_ahrs* x)
{
    //object_free(x->clock);
    ;
}

void* ahrs_new(t_symbol* s, long argc, t_atom* argv)
{
    t_ahrs* x = (t_ahrs*)object_alloc(ahrs_class);
    
    if(x){
        //x->ahrs = new FusionAhrs;
        FusionAhrsInitialise(&x->ahrs);
        
        x->dump = outlet_new(x, NULL);
        x->out3 = outlet_new(x, NULL);
        x->out2 = outlet_new(x, NULL);
        x->out = outlet_new(x, NULL);
        
        x->gain = .5;
        x->accelRejection = 10;
        x->callibrating = false;
        
        //x->clock = clock_new(x, (method)ahrs_update);
        ahrs_update(x);
        
        attr_args_process(x, argc, argv);
    }
    
    return (x);
}

void ahrs_update(t_ahrs *x){
    
    if(x->callibrating){
        ahrs_callibrate(x);
        return;
    }
    
    // Apply calibration
    x->gyro = FusionVectorSubtract(x->gyro, x->gyroOffset);
    x->accel = FusionVectorSubtract(x->accel, x->accelOffset);

    // Update gyroscope offset correction algorithm
    x->gyro = FusionOffsetUpdate(&x->offset, x->gyro);
    
    FusionAhrsUpdateNoMagnetometer(&x->ahrs, x->gyro, x->accel, x->dt);
    
    t_atom quat_out[4];
    t_atom gravity[3];
    t_atom linear_accel[3];
    
    atom_setfloat_array(4, quat_out, 4, FusionAhrsGetQuaternion(&x->ahrs).array);
    atom_setfloat_array(3, gravity, 3, FusionAhrsGetGravity(&x->ahrs).array);
    atom_setfloat_array(3, linear_accel, 3, FusionAhrsGetLinearAcceleration(&x->ahrs).array);
    
    outlet_list(x->out, gensym("list"), 4, quat_out);
    outlet_list(x->out2, gensym("list"), 3, linear_accel);
    outlet_list(x->out3, gensym("list"), 3, gravity);
}

void ahrs_callibrate(t_ahrs *x){
    t_atom a;
    atom_setfloat(&a, (float)x->callibration_samp / x->target_samps);
    outlet_anything(x->dump, gensym("callibration_progress"), 1, &a);
    if(x->callibration_samp < x->target_samps){
        x->gyroOffset = FusionVectorAdd(x->gyroOffset, x->gyro);
        x->accel = FusionVectorSubtract(x->accel, (FusionVector){0.0f, 0.0f, 1.0f});
        x->accelOffset = FusionVectorAdd(x->accelOffset, x->accel);
        x->callibration_samp++;
    }
    else if(x->callibration_samp == x->target_samps){
        x->gyroOffset = FusionVectorMultiplyScalar(x->gyroOffset, 1.0f / x->target_samps);
        //x->gyroOffset = FUSION_VECTOR_ONES;
        x->accelOffset = FusionVectorMultiplyScalar(x->accelOffset, 1.0f / x->target_samps);
        x->callibrating = false;
        //object_post((t_object *)x, "Callibration complete.");
    }
}

t_max_err ahrs_gain_set(t_ahrs* x, void* attr, long ac, t_atom* av){
    x->gain = atom_getfloat(av);
    ahrs_update_settings(x);
    return MAX_ERR_NONE;
}

t_max_err ahrs_accelRejection_set(t_ahrs* x, void* attr, long ac, t_atom* av){
    x->accelRejection = atom_getfloat(av);
    ahrs_update_settings(x);
    return MAX_ERR_NONE;
}

void ahrs_update_settings(t_ahrs* x){
    FusionAhrsSettings settings = {
        .gain = x->gain,
        .accelerationRejection = x->accelRejection
    };
    FusionAhrsSetSettings(&x->ahrs, &settings);
}

void ahrs_list(t_ahrs *x, t_symbol *s, long argc, t_atom *argv) {
    if(argc >= 6){
        for (long i = 0; i < 3; i++) {
            x->gyro.array[i] = atom_getfloat(argv + i);
        }
        for (long i = 0; i < 3; i++) {
            x->accel.array[i] = atom_getfloat(argv + 3 + i);
        }
        
        //use external delta time if provided
        if(argc >= 7){
            x->dt = atom_getfloat(argv + 6);
        }
        else{
            x->dt = (gettime() - x->prevMs) * 0.001;
            x->prevMs = gettime();
        }
        
        ahrs_update(x);
    }
}

void ahrs_anything(t_ahrs* x, t_symbol* s, long ac, t_atom* av)
{
    if (s == gensym("reset")) {
        FusionAhrsReset(&x->ahrs);
    }
    else if(s == gensym("callibratesamps") && ac > 0 && (x->target_samps = (int)atom_getlong(&av[0]))) {
        //object_post((t_object *)x, "Callibrating. Keep IMU still and on a flat surface.");
        x->callibration_samp = 0;
        x->callibrating = true;
        x->gyroOffset = FUSION_VECTOR_ZERO;
        x->accelOffset = FUSION_VECTOR_ZERO;
        //x->target_samps = atom_getfloat(&av[0]);
    }
}
