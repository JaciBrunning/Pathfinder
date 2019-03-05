#include <jni.h>
#include <stdlib.h>
#include <pathfinder.h>

#include "jaci_pathfinder_PathfinderJNI.h"

typedef void (*fitmethod)(Waypoint,Waypoint,Spline*);

void *getField(JNIEnv *env, jobject obj, char *field_name, char *field_signature) {
    jclass cls = (*env)->GetObjectClass(env, obj);
    jfieldID fid = (*env)->GetFieldID(env, cls, field_name, field_signature);
    return (*env)->GetObjectField(env, obj, fid);
}

double getDoubleField(JNIEnv *env, jobject obj, char *field_name) {
    jclass cls = (*env)->GetObjectClass(env, obj);
    jfieldID fid = (*env)->GetFieldID(env, cls, field_name, "D");
    double x = (*env)->GetDoubleField(env, obj, fid);
    return x;
}

int getIntField(JNIEnv *env, jobject obj, char *field_name) {
    jclass cls = (*env)->GetObjectClass(env, obj);
    jfieldID fid = (*env)->GetFieldID(env, cls, field_name, "I");
    int x = (*env)->GetIntField(env, obj, fid);
    return x;
}

void setDoubleField(JNIEnv *env, jobject obj, char *field_name, double value) {
    jclass cls = (*env)->GetObjectClass(env, obj);
    jfieldID fid = (*env)->GetFieldID(env, cls, field_name, "D");
    (*env)->SetDoubleField(env, obj, fid, value);
}

void setIntField(JNIEnv *env, jobject obj, char *field_name, int value) {
    jclass cls = (*env)->GetObjectClass(env, obj);
    jfieldID fid = (*env)->GetFieldID(env, cls, field_name, "I");
    (*env)->SetIntField(env, obj, fid, value);
}

void *getMethodReturn(JNIEnv *env, jobject obj, char *method_name, char *method_signature) {
    jclass cls = (*env)->GetObjectClass(env, obj);
    jmethodID mid = (*env)->GetMethodID(env, cls, method_name, method_signature);
    return (*env)->CallObjectMethod(env, obj, mid);
}

fitmethod getFitMethod(JNIEnv *env, jobject obj) {
    jstring s = (jstring) getMethodReturn(env, obj, "name", "()Ljava/lang/String;");
    const char *nativeString = (*env)->GetStringUTFChars(env, s, 0);
    
    fitmethod method;
    
    if (strcmp(nativeString, "FIT_HERMITE_CUBIC") == 0) {
        method = FIT_HERMITE_CUBIC;
    } else if (strcmp(nativeString, "FIT_HERMITE_QUINTIC") == 0) {
        method = FIT_HERMITE_QUINTIC;
    } else {
        method = FIT_HERMITE_CUBIC;
    }
    
    (*env)->ReleaseStringUTFChars(env, s, nativeString);
    return method;
}

SWERVE_MODE getSwerveMode(JNIEnv *env, jobject obj) {
    jstring s = (jstring) getMethodReturn(env, obj, "name", "()Ljava/lang/String;");
    const char *nativeString = (*env)->GetStringUTFChars(env, s, 0);
    
    SWERVE_MODE mode;
    
    if (strcmp(nativeString, "SWERVE_DEFAULT")) {
        mode = SWERVE_DEFAULT;
    } else {
        mode = SWERVE_DEFAULT;
    }
    
    (*env)->ReleaseStringUTFChars(env, s, nativeString);
    return mode;
}

jint throwJNIException(JNIEnv *env, const char *msg) {
    jclass exClass;
    char *className = "jaci/pathfinder/Pathfinder$PathfinderJNIException";
    exClass = (*env)->FindClass(env, className);
    return (*env)->ThrowNew(env, exClass, msg);
}

jint throwPathfinderException(JNIEnv *env) {
    if (pathfinder_has_error())
        return throwJNIException(env, pathfinder_error());
    return throwJNIException(env, "Unknown Exception (invalid state).");
}

jint throwGenerationException(JNIEnv *env) {
    jclass exClass;
    char *className = "jaci/pathfinder/Pathfinder$GenerationException";
    exClass = (*env)->FindClass(env, className);
    return (*env)->ThrowNew(env, exClass, "The trajectory provided was invalid! Invalid trajectory could not be generated");
}

/*
 * Class:     jaci_pathfinder_PathfinderJNI
 * Method:    generateTrajectory
 * Signature: ([Ljaci/pathfinder/Waypoint;Ljaci/pathfinder/Trajectory/FitMethod;IDDDD)[Ljaci/pathfinder/Trajectory/Segment;
 *
 * RETURN: Trajectory Object
 * PARAMS:
 *  waypoints: Array of Waypoints to generate with
 *  config:    Configuration for Trajectory Generation
 *      - fit, samples, dt, max_velocity, max_acceleration, max_jerk
 */
JNIEXPORT jobjectArray JNICALL Java_jaci_pathfinder_PathfinderJNI_generateTrajectory
    (JNIEnv *env, jclass thisCls, jobjectArray waypoints, jobject fit, jint samples, jdouble dt, jdouble max_velocity, jdouble max_acceleration, jdouble max_jerk) {
    pathfinder_clear_errors();

    int point_length = (*env)->GetArrayLength(env, waypoints);
    int i;
    Waypoint *points = malloc(point_length * sizeof(Waypoint));
    if (points == NULL) {
        pathfinder_set_error("JNI: Could not malloc waypoints.");
        return throwPathfinderException(env);
    }
    
    for (i = 0; i < point_length; i++) {
        jobject wpobj = (jobject) (*env)->GetObjectArrayElement(env, waypoints, i);
        Waypoint p = {
            getDoubleField(env, wpobj, "x"),
            getDoubleField(env, wpobj, "y"),
            getDoubleField(env, wpobj, "angle")
        };
        points[i] = p;
    }
    
    TrajectoryCandidate cd;
    fitmethod fitm = getFitMethod(env, fit);

    pathfinder_prepare(points, point_length, fitm, samples, dt, max_velocity, max_acceleration, max_jerk, &cd);
    
    if (pathfinder_has_error()) {
        return throwPathfinderException(env);
    }

    int len = cd.length;
    // Segment segs[len];
    Segment *segs = malloc(len * sizeof(Segment));
    if (segs == NULL) {
        pathfinder_set_error("JNI: Could not malloc segments.");
        free(points);
        return throwPathfinderException(env);
    }
    
    int result = pathfinder_generate(&cd, segs);
    
    if (result < 0) {
        return throwGenerationException(env);
    }
    
    jclass cls = (*env)->FindClass(env, "jaci/pathfinder/Trajectory$Segment");
    jmethodID constructor = (*env)->GetMethodID(env, cls, "<init>", "(DDDDDDDD)V");
    
    jobjectArray newArray = (*env)->NewObjectArray(env, len, cls, NULL);
    
    for (i = 0; i < len; i++) {
        Segment s = segs[i];
        jobject jseg = (*env)->NewObject(env, cls, constructor, s.dt, s.x, s.y, s.position, s.velocity, s.acceleration, s.jerk, s.heading);
        (*env)->SetObjectArrayElement(env, newArray, i, jseg);
    }
    
    free(points); free(segs);
    
    return newArray;
}

/*
 * Class:     jaci_pathfinder_PathfinderJNI
 * Method:    modifyTrajectoryTank
 * Signature: ([Ljaci/pathfinder/Trajectory/Segment;D)[[Ljaci/pathfinder/Trajectory/Segment;
 *
 * RETURN: Array of trajectories (0 = Left, 1 = Right)
 * PARAMS:
 *  source:             Source Generated Trajectory
 *  wheelbase_width:    The width between individual sides of the wheelbase
 */
JNIEXPORT jobjectArray JNICALL Java_jaci_pathfinder_PathfinderJNI_modifyTrajectoryTank
  (JNIEnv *env, jclass thisCls, jobjectArray source, jdouble wheelbase_width) {
    pathfinder_clear_errors();

    int length = (*env)->GetArrayLength(env, source);
    // Segment segs[length];
    Segment *segs = malloc(length * sizeof(Segment));
    if (segs == NULL) {
        pathfinder_set_error("JNI: Could not malloc segments.");
        return throwPathfinderException(env);
    }
    
    int i;
    for (i = 0; i < length; i++) {
        jobject sobj = (jobject) (*env)->GetObjectArrayElement(env, source, i);
        Segment s = {
            getDoubleField(env, sobj, "dt"),
            getDoubleField(env, sobj, "x"),
            getDoubleField(env, sobj, "y"),
            getDoubleField(env, sobj, "position"),
            getDoubleField(env, sobj, "velocity"),
            getDoubleField(env, sobj, "acceleration"),
            getDoubleField(env, sobj, "jerk"),
            getDoubleField(env, sobj, "heading")
        };
        segs[i] = s;
    }
    
    // Segment left[length];
    // Segment right[length];
    
    Segment *left = malloc(length * sizeof(Segment));
    if (left == NULL) {
        pathfinder_set_error("JNI: Could not malloc left segs.");
        free(segs);
        return throwPathfinderException(env);
    }
    Segment *right = malloc(length * sizeof(Segment));
    if (right == NULL) {
        pathfinder_set_error("JNI: Could not malloc right segs.");
        free(segs); free(left);
        return throwPathfinderException(env);
    }
    
    pathfinder_modify_tank(segs, length, left, right, wheelbase_width);
    
    jclass cls = (*env)->FindClass(env, "jaci/pathfinder/Trajectory$Segment");
    jmethodID constructor = (*env)->GetMethodID(env, cls, "<init>", "(DDDDDDDD)V");
    
    jobjectArray leftArray = (*env)->NewObjectArray(env, length, cls, NULL);
    jobjectArray rightArray = (*env)->NewObjectArray(env, length, cls, NULL);
    
    for (i = 0; i < length; i++) {
        Segment l = left[i];
        Segment r = right[i];
        
        jobject jlseg = (*env)->NewObject(env, cls, constructor, l.dt, l.x, l.y, l.position, l.velocity, l.acceleration, l.jerk, l.heading);
        jobject jrseg = (*env)->NewObject(env, cls, constructor, r.dt, r.x, r.y, r.position, r.velocity, r.acceleration, r.jerk, r.heading);
        
        (*env)->SetObjectArrayElement(env, leftArray, i, jlseg);
        (*env)->SetObjectArrayElement(env, rightArray, i, jrseg);
    }
    
    free(segs); free(left); free(right);
    
    cls = (*env)->FindClass(env, "[Ljaci/pathfinder/Trajectory$Segment;");
    jobjectArray returnArray = (*env)->NewObjectArray(env, 2, cls, NULL);
    
    (*env)->SetObjectArrayElement(env, returnArray, 0, leftArray);
    (*env)->SetObjectArrayElement(env, returnArray, 1, rightArray);
    
    return returnArray;
}

/*
 * Class:     jaci_pathfinder_PathfinderJNI
 * Method:    modifyTrajectorySwerve
 * Signature: ([Ljaci/pathfinder/Trajectory/Segment;DDLjaci/pathfinder/modifiers/SwerveModifier/Mode;)[[Ljaci/pathfinder/Trajectory/Segment;
 * 
 * RETURN: Array of trajectories (0 = Front Left, 1 = Front Right, 2 = Back Left, 3 = Back Right)
 * PARAMS:
 *  source:             Source Generated Trajectory
 *  wheelbase_width:    The width between individual sides of the wheelbase
 *  wheelbase_depth:    The depth between the front and back of the wheelbases
 *  mode:               The mode to use to generate the new trajectories
 */
JNIEXPORT jobjectArray JNICALL Java_jaci_pathfinder_PathfinderJNI_modifyTrajectorySwerve
  (JNIEnv *env, jclass thisCls, jobjectArray source, jdouble wheelbase_width, jdouble wheelbase_depth, jobject mode) {
    pathfinder_clear_errors();

    int length = (*env)->GetArrayLength(env, source);
    // Segment segs[length];
    Segment *segs = malloc(length * sizeof(Segment));
    if (segs == NULL) {
        pathfinder_set_error("JNI: Could not malloc segments.");
        return throwPathfinderException(env);
    }
    
    int i;
    for (i = 0; i < length; i++) {
        jobject sobj = (jobject) (*env)->GetObjectArrayElement(env, source, i);
        Segment s = {
            getDoubleField(env, sobj, "dt"),
            getDoubleField(env, sobj, "x"),
            getDoubleField(env, sobj, "y"),
            getDoubleField(env, sobj, "position"),
            getDoubleField(env, sobj, "velocity"),
            getDoubleField(env, sobj, "acceleration"),
            getDoubleField(env, sobj, "jerk"),
            getDoubleField(env, sobj, "heading")
        };
        segs[i] = s;
    }
    
    // Segment fl[length];
    // Segment fr[length];
    // Segment bl[length];
    // Segment br[length];
    
    Segment *fl = malloc(length * sizeof(Segment));
    if (fl == NULL) {
        pathfinder_set_error("JNI: Could not malloc fl segs.");
        free(segs);
        return throwPathfinderException(env);
    }
    Segment *fr = malloc(length * sizeof(Segment));
    if (fr == NULL) {
        pathfinder_set_error("JNI: Could not malloc fr segs.");
        free(segs); free(fl);
        return throwPathfinderException(env);
    }
    Segment *bl = malloc(length * sizeof(Segment));
    if (bl == NULL) {
        pathfinder_set_error("JNI: Could not malloc bl segs.");
        free(segs); free(fl); free(bl);
        return throwPathfinderException(env);
    }
    Segment *br = malloc(length * sizeof(Segment));
    if (br == NULL) {
        pathfinder_set_error("JNI: Could not malloc br segs.");
        free(segs); free(fl); free(bl); free(br);
        return throwPathfinderException(env);
    }
    
    SWERVE_MODE smode = getSwerveMode(env, mode);
    pathfinder_modify_swerve(segs, length, fl, fr, bl, br, wheelbase_width, wheelbase_depth, smode);
    
    jclass cls = (*env)->FindClass(env, "jaci/pathfinder/Trajectory$Segment");
    jmethodID constructor = (*env)->GetMethodID(env, cls, "<init>", "(DDDDDDDD)V");
    
    jobjectArray fla = (*env)->NewObjectArray(env, length, cls, NULL);
    jobjectArray fra = (*env)->NewObjectArray(env, length, cls, NULL);
    jobjectArray bla = (*env)->NewObjectArray(env, length, cls, NULL);
    jobjectArray bra = (*env)->NewObjectArray(env, length, cls, NULL);
    
    for (i = 0; i < length; i++) {
        Segment sfl = fl[i];
        Segment sfr = fr[i];
        Segment sbl = bl[i];
        Segment sbr = br[i];
        
        jobject jfl = (*env)->NewObject(env, cls, constructor, sfl.dt, sfl.x, sfl.y, sfl.position, sfl.velocity, sfl.acceleration, sfl.jerk, sfl.heading);
        jobject jfr = (*env)->NewObject(env, cls, constructor, sfr.dt, sfr.x, sfr.y, sfr.position, sfr.velocity, sfr.acceleration, sfr.jerk, sfr.heading);
        jobject jbl = (*env)->NewObject(env, cls, constructor, sbl.dt, sbl.x, sbl.y, sbl.position, sbl.velocity, sbl.acceleration, sbl.jerk, sbl.heading);
        jobject jbr = (*env)->NewObject(env, cls, constructor, sbr.dt, sbr.x, sbr.y, sbr.position, sbr.velocity, sbr.acceleration, sbr.jerk, sbr.heading);
        
        (*env)->SetObjectArrayElement(env, fla, i, jfl);
        (*env)->SetObjectArrayElement(env, fra, i, jfr);
        (*env)->SetObjectArrayElement(env, bla, i, jbl);
        (*env)->SetObjectArrayElement(env, bra, i, jbr);
    }
    
    free(segs); free(fl); free(fr); free(bl); free(br);
    
    cls = (*env)->FindClass(env, "[Ljaci/pathfinder/Trajectory$Segment;");
    jobjectArray returnArray = (*env)->NewObjectArray(env, 4, cls, NULL);
    
    (*env)->SetObjectArrayElement(env, returnArray, 0, fla);
    (*env)->SetObjectArrayElement(env, returnArray, 1, fra);
    (*env)->SetObjectArrayElement(env, returnArray, 2, bla);
    (*env)->SetObjectArrayElement(env, returnArray, 3, bra);
    
    return returnArray;
}

/*
 * Class:     jaci_pathfinder_PathfinderJNI
 * Method:    trajectorySerialize
 * Signature: ([Ljaci/pathfinder/Trajectory/Segment;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_jaci_pathfinder_PathfinderJNI_trajectorySerialize
  (JNIEnv *env, jclass thisCls, jobjectArray trajectory, jstring filename) {
    pathfinder_clear_errors();

    int length = (*env)->GetArrayLength(env, trajectory);
    Segment *segs = malloc(length * sizeof(Segment));
    if (segs == NULL) {
        pathfinder_set_error("JNI: Could not malloc segments");
        throwPathfinderException(env);
        return;
    }
    
    int i;
    for (i = 0; i < length; i++) {
        jobject sobj = (jobject) (*env)->GetObjectArrayElement(env, trajectory, i);
        Segment s = {
            getDoubleField(env, sobj, "dt"),
            getDoubleField(env, sobj, "x"),
            getDoubleField(env, sobj, "y"),
            getDoubleField(env, sobj, "position"),
            getDoubleField(env, sobj, "velocity"),
            getDoubleField(env, sobj, "acceleration"),
            getDoubleField(env, sobj, "jerk"),
            getDoubleField(env, sobj, "heading")
        };
        segs[i] = s;
    }
    
    const char *path;
    path = (*env)->GetStringUTFChars( env, filename, 0 ) ;
    FILE *fp = fopen(path, "wb");
    
    // This catches an invalid fp
    if (pathfinder_serialize(fp, segs, length) < 0) {
        throwPathfinderException(env);
        return;
    }
    
    fclose(fp);
    (*env)->ReleaseStringUTFChars(env, filename, path);
}

/*
 * Class:     jaci_pathfinder_PathfinderJNI
 * Method:    trajectoryDeserialize
 * Signature: (Ljava/lang/String;)[Ljaci/pathfinder/Trajectory/Segment;
 */
JNIEXPORT jobjectArray JNICALL Java_jaci_pathfinder_PathfinderJNI_trajectoryDeserialize
  (JNIEnv *env, jclass thisCls, jstring filename) {
    pathfinder_clear_errors();
    const char *path;
    path = (*env)->GetStringUTFChars( env, filename, NULL ) ;
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        pathfinder_set_error("JNI: Invalid file path.");
        return throwPathfinderException(env);
    }

    int num_lines = pathfinder_get_file_length(fp);
    if (num_lines < 0) {
        return throwPathfinderException(env);
    }

    Segment *segs = malloc(num_lines * sizeof(Segment));
    if (segs == NULL) {
        pathfinder_set_error("JNI: Could not malloc segments");
        return throwPathfinderException(env);
    }

    int length = pathfinder_deserialize(fp, segs);

    if (length < 0) {
        return throwPathfinderException(env);
    }
    
    fclose(fp);
    (*env)->ReleaseStringUTFChars(env, filename, path);
    
    jclass cls = (*env)->FindClass(env, "jaci/pathfinder/Trajectory$Segment");
    jmethodID constructor = (*env)->GetMethodID(env, cls, "<init>", "(DDDDDDDD)V");
    jobjectArray arr = (*env)->NewObjectArray(env, length, cls, NULL);
    
    int i;
    for (i = 0; i < length; i++) {
        Segment s = segs[i];
        jobject js = (*env)->NewObject(env, cls, constructor, s.dt, s.x, s.y, s.position, s.velocity, s.acceleration, s.jerk, s.heading);
        (*env)->SetObjectArrayElement(env, arr, i, js);
    }
    
    free(segs);
    return arr;
}

/*
 * Class:     jaci_pathfinder_PathfinderJNI
 * Method:    trajectorySerializeCSV
 * Signature: ([Ljaci/pathfinder/Trajectory/Segment;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_jaci_pathfinder_PathfinderJNI_trajectorySerializeCSV
  (JNIEnv *env, jclass thiscls, jobjectArray trajectory, jstring filename) {
    pathfinder_clear_errors();
    int length = (*env)->GetArrayLength(env, trajectory);
    Segment *segs = malloc(length * sizeof(Segment));
    if (segs == NULL) {
        pathfinder_set_error("JNI: Could not malloc segments");
        throwPathfinderException(env);
        return;
    }
    
    int i;
    for (i = 0; i < length; i++) {
        jobject sobj = (jobject) (*env)->GetObjectArrayElement(env, trajectory, i);
        Segment s = {
            getDoubleField(env, sobj, "dt"),
            getDoubleField(env, sobj, "x"),
            getDoubleField(env, sobj, "y"),
            getDoubleField(env, sobj, "position"),
            getDoubleField(env, sobj, "velocity"),
            getDoubleField(env, sobj, "acceleration"),
            getDoubleField(env, sobj, "jerk"),
            getDoubleField(env, sobj, "heading")
        };
        segs[i] = s;
    }
    
    const char *path;
    path = (*env)->GetStringUTFChars( env, filename, 0 ) ;
    FILE *fp = fopen(path, "w");
    
    if (pathfinder_serialize_csv(fp, segs, length) < 0) {
        throwPathfinderException(env);
        return;
    }
    
    fclose(fp);
    (*env)->ReleaseStringUTFChars(env, filename, path);
}

/*
 * Class:     jaci_pathfinder_PathfinderJNI
 * Method:    trajectoryDeserializeCSV
 * Signature: (Ljava/lang/String;)[Ljaci/pathfinder/Trajectory/Segment;
 */
JNIEXPORT jobjectArray JNICALL Java_jaci_pathfinder_PathfinderJNI_trajectoryDeserializeCSV
  (JNIEnv *env, jclass thiscls, jstring filename) {
    pathfinder_clear_errors();
    const char *path;
    path = (*env)->GetStringUTFChars( env, filename, NULL ) ;
    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        pathfinder_set_error("JNI: Invalid file path.");
        return throwPathfinderException(env);
    }

    int num_lines = pathfinder_get_file_length(fp);
    if (num_lines < 0) {
        return throwPathfinderException(env);
    }

    Segment *segs = malloc(num_lines * sizeof(Segment));  // Pre allocate for a huge amount of segments
    if (segs == NULL) {
        pathfinder_set_error("JNI: Could not malloc segments");
        return throwPathfinderException(env);
    }

    int length = pathfinder_deserialize_csv(fp, segs);
    if (length < 0) {
        return throwPathfinderException(env);
    }
    
    fclose(fp);
    (*env)->ReleaseStringUTFChars(env, filename, path);
    
    jclass cls = (*env)->FindClass(env, "jaci/pathfinder/Trajectory$Segment");
    jmethodID constructor = (*env)->GetMethodID(env, cls, "<init>", "(DDDDDDDD)V");
    jobjectArray arr = (*env)->NewObjectArray(env, length, cls, NULL);
    
    int i;
    for (i = 0; i < length; i++) {
        Segment s = segs[i];
        jobject js = (*env)->NewObject(env, cls, constructor, s.dt, s.x, s.y, s.position, s.velocity, s.acceleration, s.jerk, s.heading);
        (*env)->SetObjectArrayElement(env, arr, i, js);
    }
    
    free(segs);
    return arr;
}