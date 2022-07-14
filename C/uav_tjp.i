/* File: example.i */
%include std_string.i
%module uav_tjp

%{
    #define SWIG_FILE_WITH_INIT
    #include "uav_tjp.h"
    #include "point.h"
    using namespace std;
%}

%include "uav_tjp.h"
%include "point.h"
