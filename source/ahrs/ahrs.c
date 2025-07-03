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
    
    void *out;
    void *out2;
    void *out3;
} t_ahrs;

//prototype
void* ahrs_new(t_symbol* s, long argc, t_atom* argv);
void ahrs_free(t_ahrs* x);
void ahrs_assist(t_ahrs* x, void* b, long m, long a, char* s);
void ahrs_list(t_ahrs *x, t_symbol *s, long argc, t_atom *argv);
void ahrs_anything(t_ahrs* x, t_symbol* s, long ac, t_atom* av);
void ahrs_update(t_ahrs *x);

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
    
    CLASS_ATTR_DOUBLE(c, "accelerationRejection", 0, t_ahrs, accelRejection);
    CLASS_ATTR_ACCESSORS(c, "accelerationRejection", NULL, (method)ahrs_accelRejection_set)
    CLASS_ATTR_LABEL(c, "accelerationRejection", 0, "Acceleration rejection");
    
    class_register(CLASS_BOX, c);
    ahrs_class = c;
}

void ahrs_assist(t_ahrs* x, void* b, long m, long a, char* s)
{
    if (m == ASSIST_INLET) { // inlet
        sprintf(s, "I am inlet %ld", a);
    }
    else { // outlet
        sprintf(s, "I am outlet %ld", a);
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
        
        x->out3 = outlet_new(x, NULL);
        x->out2 = outlet_new(x, NULL);
        x->out = outlet_new(x, NULL);
        
        x->gain = .5;
        x->accelRejection = 10;
        
        //x->clock = clock_new(x, (method)ahrs_update);
        ahrs_update(x);
        
        attr_args_process(x, argc, argv);
    }
    
    return (x);
}

// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

void ahrs_update(t_ahrs *x){
    //post("tick");
    
    // Apply calibration
    x->gyro = FusionCalibrationInertial(x->gyro, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    x->accel = FusionCalibrationInertial(x->accel, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

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
    //post("Received list on left inlet with %ld items:", argc);
    
    if(argc >= 6){
        for (long i = 0; i < 3; i++) {
            x->gyro.array[i] = atom_getfloat(argv + i);
        }
        for (long i = 0; i < 3; i++) {
            x->accel.array[i] = atom_getfloat(argv + 3 + i);
        }
        
        if(argc == 7){
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
}
