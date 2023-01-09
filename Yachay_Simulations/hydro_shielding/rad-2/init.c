/* ///////////////////////////////////////////////////////////////////// */
/*! 
  \file  
  \brief Contains basic functions for problem initialization.

  The init.c file collects most of the user-supplied functions useful 
  for problem configuration.
  It is automatically searched for by the makefile.

  \author A. Mignone (mignone@ph.unito.it)
  \date   March 5, 2017
*/
/* ///////////////////////////////////////////////////////////////////// */
#include "pluto.h"

/* ********************************************************************* */
void Init (double *v, double x1, double x2, double x3)
/*! 
 * The Init() function can be used to assign initial conditions as
 * as a function of spatial position.
 *
 * \param [out] v   a pointer to a vector of primitive variables
 * \param [in] x1   coordinate point in the 1st dimension
 * \param [in] x2   coordinate point in the 2nd dimension
 * \param [in] x3   coordinate point in the 3rdt dimension
 *
 * The meaning of x1, x2 and x3 depends on the geometry:
 * \f[ \begin{array}{cccl}
 *    x_1  & x_2    & x_3  & \mathrm{Geometry}    \\ \noalign{\medskip}
 *     \hline
 *    x    &   y    &  z   & \mathrm{Cartesian}   \\ \noalign{\medskip}
 *    R    &   z    &  -   & \mathrm{cylindrical} \\ \noalign{\medskip}
 *    R    & \phi   &  z   & \mathrm{polar}       \\ \noalign{\medskip}
 *    r    & \theta & \phi & \mathrm{spherical} 
 *    \end{array}
 *  \f]
 *
 * Variable names are accessed by means of an index v[nv], where
 * nv = RHO is density, nv = PRS is pressure, nv = (VX1, VX2, VX3) are
 * the three components of velocity, and so forth.
 *
 *********************************************************************** */
{
double r, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15, r16, r17, r18, r19, r20, r21, r22, r23, r24, r25, r26, r27, r28, r29, r30, r31, r32, r33, r34, x0, y0, z0, delta; 
// r  -> cloud radius
// x0, y0, z0 -> coordinates of the centre

x0 = 0.;
y0 = 0.;
z0 = 0.;

// Define gamma for monoatic gas
g_gamma = 5./3.;

// Cooling floor in Kelvin
// This is adding heating artificially to the gas
g_minCoolingTemp = 1.e4;

// Cloud separation:
delta = g_inputParam[DELTA];

// Cloud radius:
r = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - y0)*(x2 - y0), + (x3 - z0)*(x3 - z0));

r1 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-delta))*(x2 - (y0-delta)), + (x3 - z0)*(x3 - z0));
r2 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+delta))*(x2 - (y0+delta)), + (x3 - z0)*(x3 - z0));
r3 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-2*delta))*(x2 - (y0-2*delta)), + (x3 - z0)*(x3 - z0));
r4 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+2*delta))*(x2 - (y0+2*delta)), + (x3 - z0)*(x3 - z0));

r5 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-4*delta))*(x2 - (y0-4*delta)), + (x3 - z0)*(x3 - z0));
r6 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+4*delta))*(x2 - (y0+4*delta)), + (x3 - z0)*(x3 - z0));
r7 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-8*delta))*(x2 - (y0-8*delta)), + (x3 - z0)*(x3 - z0));
r8 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+8*delta))*(x2 - (y0+8*delta)), + (x3 - z0)*(x3 - z0));

r9= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-3*delta))*(x2 - (y0-3*delta)), + (x3 - z0)*(x3 - z0));
r10 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+3*delta))*(x2 - (y0+3*delta)), + (x3 - z0)*(x3 - z0));

r11= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-5*delta))*(x2 - (y0-5*delta)), + (x3 - z0)*(x3 - z0));
r12 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+5*delta))*(x2 - (y0+5*delta)), + (x3 - z0)*(x3 - z0));

r13= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-6*delta))*(x2 - (y0-6*delta)), + (x3 - z0)*(x3 - z0));
r14 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+6*delta))*(x2 - (y0+6*delta)), + (x3 - z0)*(x3 - z0));

r15= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-7*delta))*(x2 - (y0-7*delta)), + (x3 - z0)*(x3 - z0));
r16 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+7*delta))*(x2 - (y0+7*delta)), + (x3 - z0)*(x3 - z0));

r17= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-8*delta))*(x2 - (y0-8*delta)), + (x3 - z0)*(x3 - z0));
r18 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+8*delta))*(x2 - (y0+8*delta)), + (x3 - z0)*(x3 - z0));

r19= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-9*delta))*(x2 - (y0-9*delta)), + (x3 - z0)*(x3 - z0));
r20 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+9*delta))*(x2 - (y0+9*delta)), + (x3 - z0)*(x3 - z0));

r21= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-10*delta))*(x2 - (y0-10*delta)), + (x3 - z0)*(x3 - z0));
r22 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+10*delta))*(x2 - (y0+10*delta)), + (x3 - z0)*(x3 - z0));

r23= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-11*delta))*(x2 - (y0-11*delta)), + (x3 - z0)*(x3 - z0));
r24 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+11*delta))*(x2 - (y0+11*delta)), + (x3 - z0)*(x3 - z0));

r25= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-12*delta))*(x2 - (y0-12*delta)), + (x3 - z0)*(x3 - z0));
r26 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+12*delta))*(x2 - (y0+12*delta)), + (x3 - z0)*(x3 - z0));

r27= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-13*delta))*(x2 - (y0-13*delta)), + (x3 - z0)*(x3 - z0));
r28 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+13*delta))*(x2 - (y0+13*delta)), + (x3 - z0)*(x3 - z0));

r29= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-14*delta))*(x2 - (y0-14*delta)), + (x3 - z0)*(x3 - z0));
r30 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+14*delta))*(x2 - (y0+14*delta)), + (x3 - z0)*(x3 - z0));

r31= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-15*delta))*(x2 - (y0-15*delta)), + (x3 - z0)*(x3 - z0));
r32 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+15*delta))*(x2 - (y0+15*delta)), + (x3 - z0)*(x3 - z0));

r33= DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0-16*delta))*(x2 - (y0-16*delta)), + (x3 - z0)*(x3 - z0));
r34 = DIM_EXPAND((x1 - x0)*(x1 - x0), + (x2 - (y0+16*delta))*(x2 - (y0+16*delta)), + (x3 - z0)*(x3 - z0));



// Cloud parameters:
if (sqrt(r) < g_inputParam[RADIUS]) {
  v[RHO] = 100.0;
  v[VX1] = 0.0;
  v[VX2] = 0.0;
  v[VX3] = 0.0;
  v[TRC] = 1.0;
  v[TRC+1] = 1.0;
}
else if ((sqrt(r1) < g_inputParam[RADIUS]) || (sqrt(r2) < g_inputParam[RADIUS]) || (sqrt(r3) < g_inputParam[RADIUS]) || (sqrt(r4) < g_inputParam[RADIUS]) || (sqrt(r5) < g_inputParam[RADIUS]) || (sqrt(r6) < g_inputParam[RADIUS]) || (sqrt(r7) < g_inputParam[RADIUS]) || (sqrt(r8) < g_inputParam[RADIUS])   || (sqrt(r9)< g_inputParam[RADIUS]) || (sqrt(r10) < g_inputParam[RADIUS]) || (sqrt(r11) < g_inputParam[RADIUS]) || (sqrt(r12) < g_inputParam[RADIUS]) || (sqrt(r13) < g_inputParam[RADIUS]) || (sqrt(r14) < g_inputParam[RADIUS]) || (sqrt(r15) < g_inputParam[RADIUS]) || (sqrt(r16) < g_inputParam[RADIUS]) || (sqrt(r16) < g_inputParam[RADIUS]) || (sqrt(r17) < g_inputParam[RADIUS]) || (sqrt(r18) < g_inputParam[RADIUS]) || (sqrt(r19) < g_inputParam[RADIUS]) || (sqrt(r20) < g_inputParam[RADIUS]) || (sqrt(r21) < g_inputParam[RADIUS]) || (sqrt(r22) < g_inputParam[RADIUS]) || (sqrt(r23) < g_inputParam[RADIUS])  || (sqrt(r24) < g_inputParam[RADIUS]) || (sqrt(r25) < g_inputParam[RADIUS]) || (sqrt(r26) < g_inputParam[RADIUS]) || (sqrt(r27) < g_inputParam[RADIUS]) || (sqrt(r28) < g_inputParam[RADIUS]) || (sqrt(r29) < g_inputParam[RADIUS]) || (sqrt(r30) < g_inputParam[RADIUS]) || (sqrt(r31) < g_inputParam[RADIUS]) || (sqrt(r32) < g_inputParam[RADIUS]) || (sqrt(r33) < g_inputParam[RADIUS]) || (sqrt(r34) < g_inputParam[RADIUS]) ){
  v[RHO] = 100.0;
  v[VX1] = 0.0;
  v[VX2] = 0.0;
  v[VX3] = 0.0;
  v[TRC] = 1.0;
  v[TRC+1] = 0.0;
}
else{
// Wind parameters:
  v[RHO] = 1.0;
  v[VX1] = 0.0;
  v[VX2] = 5.0;
  v[VX3] = 0.0;
  v[TRC] = 0.0;
  v[TRC+1] = 0.0;
}

  #if HAVE_ENERGY
  v[PRS] = (0.01*1.e6*CONST_kB)/((0.67*CONST_amu*0.01)*(1.e7)*(1.e7));
  #endif

  #if PHYSICS == MHD || PHYSICS == RMHD
  v[BX1] = 0.0;
  v[BX2] = 0.0;
  v[BX3] = 0.0;

  v[AX1] = 0.0;
  v[AX2] = 0.0;
  v[AX3] = 0.0;
  #endif
}

/* ********************************************************************* */
void InitDomain (Data *d, Grid *grid)
/*! 
 * Assign initial condition by looping over the computational domain.
 * Called after the usual Init() function to assign initial conditions
 * on primitive variables.
 * Value assigned here will overwrite those prescribed during Init().
 *
 *
 *********************************************************************** */
{
}

/* ********************************************************************* */
void Analysis (const Data *d, Grid *grid)
/*! 
 *  Perform runtime data analysis.
 *
 * \param [in] d the PLUTO Data structure
 * \param [in] grid   pointer to array of Grid structures  
 *
 *********************************************************************** */
{

}
#if PHYSICS == MHD
/* ********************************************************************* */
void BackgroundField (double x1, double x2, double x3, double *B0)
/*!
 * Define the component of a static, curl-free background 
 * magnetic field.
 *
 * \param [in] x1  position in the 1st coordinate direction \f$x_1\f$
 * \param [in] x2  position in the 2nd coordinate direction \f$x_2\f$
 * \param [in] x3  position in the 3rd coordinate direction \f$x_3\f$
 * \param [out] B0 array containing the vector componens of the background
 *                 magnetic field
 *********************************************************************** */
{
   B0[0] = 0.0;
   B0[1] = 0.0;
   B0[2] = 0.0;
}
#endif

/* ********************************************************************* */
void UserDefBoundary (const Data *d, RBox *box, int side, Grid *grid) 
/*! 
 *  Assign user-defined boundary conditions.
 *
 * \param [in,out] d  pointer to the PLUTO data structure containing
 *                    cell-centered primitive quantities (d->Vc) and 
 *                    staggered magnetic fields (d->Vs, when used) to 
 *                    be filled.
 * \param [in] box    pointer to a RBox structure containing the lower
 *                    and upper indices of the ghost zone-centers/nodes
 *                    or edges at which data values should be assigned.
 * \param [in] side   specifies the boundary side where ghost zones need
 *                    to be filled. It can assume the following 
 *                    pre-definite values: X1_BEG, X1_END,
 *                                         X2_BEG, X2_END, 
 *                                         X3_BEG, X3_END.
 *                    The special value side == 0 is used to control
 *                    a region inside the computational domain.
 * \param [in] grid  pointer to an array of Grid structures.
 *
 *********************************************************************** */
{
  int   i, j, k, nv;
  double  *x1, *x2, *x3;

  x1 = grid->x[IDIR];
  x2 = grid->x[JDIR];
  x3 = grid->x[KDIR];

  if (side == 0) {    /* -- check solution inside domain -- */
    DOM_LOOP(k,j,i){}
  }

  if (side == X1_BEG){  /* -- X1_BEG boundary -- */
    if (box->vpos == CENTER) {
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X1FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X2FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X3FACE){
      BOX_LOOP(box,k,j,i){  }
    }
  }

  if (side == X1_END){  /* -- X1_END boundary -- */
    if (box->vpos == CENTER) {
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X1FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X2FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X3FACE){
      BOX_LOOP(box,k,j,i){  }
    }
  }

  if (side == X2_BEG){  /* -- X2_BEG boundary -- */
    if (box->vpos == CENTER) {
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X1FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X2FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X3FACE){
      BOX_LOOP(box,k,j,i){  }
    }
  }

  if (side == X2_END){  /* -- X2_END boundary -- */
    if (box->vpos == CENTER) {
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X1FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X2FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X3FACE){
      BOX_LOOP(box,k,j,i){  }
    }
  }

  if (side == X3_BEG){  /* -- X3_BEG boundary -- */
    if (box->vpos == CENTER) {
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X1FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X2FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X3FACE){
      BOX_LOOP(box,k,j,i){  }
    }
  }

  if (side == X3_END){  /* -- X3_END boundary -- */
    if (box->vpos == CENTER) {
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X1FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X2FACE){
      BOX_LOOP(box,k,j,i){  }
    }else if (box->vpos == X3FACE){
      BOX_LOOP(box,k,j,i){  }
    }
  }
}

#if BODY_FORCE != NO
/* ********************************************************************* */
void BodyForceVector(double *v, double *g, double x1, double x2, double x3)
/*!
 * Prescribe the acceleration vector as a function of the coordinates
 * and the vector of primitive variables *v.
 *
 * \param [in] v  pointer to a cell-centered vector of primitive 
 *                variables
 * \param [out] g acceleration vector
 * \param [in] x1  position in the 1st coordinate direction \f$x_1\f$
 * \param [in] x2  position in the 2nd coordinate direction \f$x_2\f$
 * \param [in] x3  position in the 3rd coordinate direction \f$x_3\f$
 *
 *********************************************************************** */
{
  g[IDIR] = 0.0;
  g[JDIR] = 0.0;
  g[KDIR] = 0.0;
}
/* ********************************************************************* */
double BodyForcePotential(double x1, double x2, double x3)
/*!
 * Return the gravitational potential as function of the coordinates.
 *
 * \param [in] x1  position in the 1st coordinate direction \f$x_1\f$
 * \param [in] x2  position in the 2nd coordinate direction \f$x_2\f$
 * \param [in] x3  position in the 3rd coordinate direction \f$x_3\f$
 * 
 * \return The body force potential \f$ \Phi(x_1,x_2,x_3) \f$.
 *
 *********************************************************************** */
{
  return 0.0;
}
#endif
