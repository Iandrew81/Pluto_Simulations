#define  PHYSICS                        HD
#define  DIMENSIONS                     3
#define  GEOMETRY                       CARTESIAN
#define  BODY_FORCE                     NO
#define  COOLING                        TABULATED
#define  RECONSTRUCTION                 PARABOLIC
#define  TIME_STEPPING                  RK3
#define  NTRACER                        2
#define  PARTICLES                      NO
#define  USER_DEF_PARAMETERS            2

/* -- physics dependent declarations -- */

#define  DUST_FLUID                     NO
#define  EOS                            IDEAL
#define  ENTROPY_SWITCH                 NO
#define  THERMAL_CONDUCTION             NO
#define  VISCOSITY                      NO
#define  ROTATING_FRAME                 NO

/* -- user-defined parameters (labels) -- */

#define  RADIUS                         0
#define  DELTA                          1

/* [Beg] user-defined constants (do not change this line) */

#define  UNIT_DENSITY                   (0.67*CONST_amu*0.01)
#define  UNIT_LENGTH                    (50*CONST_pc)
#define  UNIT_VELOCITY                  (1.e7)

/* [End] user-defined constants (do not change this line) */
