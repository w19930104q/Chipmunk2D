// Impulse-reporting semantics tests for cpDampedRotarySpring (with cpDampedSpring
// as the in-library reference for the reporting convention).
//
// Build (from the repository root):
//   cc -Iinclude src/*.c test/cpDampedRotarySpringImpulseTest.c -lm -o /tmp/impulse_test
// Run:
//   /tmp/impulse_test [trajectory.csv]
//
// If a trajectory file path is given, the per-frame dynamic state of every
// scenario is appended to it in %.17g precision so that trajectories can be
// diffed frame-by-frame between builds (the impulse-reporting fix must not
// change the dynamics).
//
// Documented semantics under test (see cpDampedRotarySpring.h):
//   cpConstraintGetImpulse() returns the TOTAL angular impulse J applied by the
//   constraint during the last cpSpaceStep(), including both the spring torque
//   term and the damping term, expressed as the impulse applied to body b
//   (-J applied to body a):
//       J = springTorqueFunc(relativeAngle)*dt - (damping impulses)
//   Verifiable identity (no other torques acting):
//       J == I_b*(w_b_after - w_b_before) == -I_a*(w_a_after - w_a_before)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "chipmunk/chipmunk.h"

static int g_failures = 0;
static FILE *g_trace = NULL;
static cpFloat g_dt;

#define CHECK_CLOSE_REL(actual, expected, relTol, label, step) do { \
		cpFloat _a = (actual), _e = (expected); \
		cpFloat _scale = fmax(fmax(fabs(_a), fabs(_e)), 1e-6); \
		if(!(fabs(_a - _e) <= (relTol)*_scale)){ \
			g_failures++; \
			printf("FAIL [%s] step %d: actual=%.10g expected=%.10g (rel err %.3g > %.3g)\n", \
				label, (int)(step), (double)_a, (double)_e, \
				(double)(fabs(_a - _e)/_scale), (double)(relTol)); \
		} \
	} while(0)

// Accumulators filled by the preSolve callback, which runs immediately before
// the constraint's preStep() with the exact same body angles/positions.
static cpFloat g_stepSpringImpulse;  // springTorque*dt of the current step
static cpFloat g_sumSpringImpulse;   // Sigma(springTorque*dt) over the scenario

static void
rotaryPreSolve(cpConstraint *constraint, cpSpace *space)
{
	cpBody *a = cpConstraintGetBodyA(constraint);
	cpBody *b = cpConstraintGetBodyB(constraint);
	cpDampedRotarySpringTorqueFunc torqueFunc = cpDampedRotarySpringGetSpringTorqueFunc(constraint);
	cpFloat relAngle = cpBodyGetAngle(a) - cpBodyGetAngle(b);
	g_stepSpringImpulse = torqueFunc(constraint, relAngle)*g_dt;
	g_sumSpringImpulse += g_stepSpringImpulse;
}

// Pure spring (damping = 0): reported impulse must equal Sigma(springTorque*dt).
static void
test_pure_spring(void)
{
	cpSpace *space = cpSpaceNew();
	cpSpaceSetGravity(space, cpvzero);
	cpSpaceSetDamping(space, 1.0f);

	cpFloat moment = 2.0f;
	cpBody *A = cpSpaceAddBody(space, cpBodyNew(1.0f, moment));
	cpBody *B = cpSpaceAddBody(space, cpBodyNew(1.0f, moment));
	cpBodySetAngle(B, 0.5f);

	cpConstraint *spring = cpDampedRotarySpringNew(A, B, 0.0f, 100.0f, 0.0f);
	cpSpaceAddConstraint(space, spring);
	cpConstraintSetPreSolveFunc(spring, rotaryPreSolve);

	g_sumSpringImpulse = 0.0f;
	cpFloat sumReported = 0.0f;
	int N = 60;
	for(int i = 0; i < N; i++){
		cpSpaceStep(space, g_dt);
		cpFloat J = cpConstraintGetImpulse(spring);
		sumReported += J;
		// Per-step: with damping = 0 the report must be exactly springTorque*dt.
		CHECK_CLOSE_REL(J, g_stepSpringImpulse, 1e-5, "pure-spring per-step", i);
		if(g_trace) fprintf(g_trace, "pure,%d,%.17g,%.17g,%.17g,%.17g\n", i,
			(double)cpBodyGetAngle(A), (double)cpBodyGetAngle(B),
			(double)cpBodyGetAngularVelocity(A), (double)cpBodyGetAngularVelocity(B));
	}

	// Acceptance: |GetImpulse sum - Sigma(springTorque*dt)| <= 1e-3 relative.
	CHECK_CLOSE_REL(sumReported, g_sumSpringImpulse, 1e-3, "pure-spring total", N);
	printf("pure-spring: Sigma(reported)=%.10g Sigma(torque*dt)=%.10g\n",
		(double)sumReported, (double)g_sumSpringImpulse);

	cpSpaceFree(space);
	cpConstraintFree(spring);
	cpBodyFree(A);
	cpBodyFree(B);
}

// Damped rotary spring: reported impulse must satisfy the angular
// impulse-momentum identity (i.e. it includes the damping term, with the same
// sign convention as the spring term).
static void
test_damped_rotary(void)
{
	cpSpace *space = cpSpaceNew();
	cpSpaceSetGravity(space, cpvzero);
	cpSpaceSetDamping(space, 1.0f);

	cpFloat moment = 2.0f;
	cpBody *A = cpSpaceAddBody(space, cpBodyNew(1.0f, moment));
	cpBody *B = cpSpaceAddBody(space, cpBodyNew(1.0f, moment));
	cpBodySetAngle(B, 0.5f);

	cpFloat damping = 5.0f;
	cpConstraint *spring = cpDampedRotarySpringNew(A, B, 0.0f, 100.0f, damping);
	cpSpaceAddConstraint(space, spring);
	cpConstraintSetPreSolveFunc(spring, rotaryPreSolve);

	g_sumSpringImpulse = 0.0f;
	cpFloat sumReported = 0.0f, sumDampImpulse = 0.0f;
	int N = 60;
	for(int i = 0; i < N; i++){
		cpFloat wa0 = cpBodyGetAngularVelocity(A);
		cpFloat wb0 = cpBodyGetAngularVelocity(B);

		cpSpaceStep(space, g_dt);
		cpFloat J = cpConstraintGetImpulse(spring);
		sumReported += J;

		cpFloat wa1 = cpBodyGetAngularVelocity(A);
		cpFloat wb1 = cpBodyGetAngularVelocity(B);

		// Documented identity: J == I_b*dw_b == -I_a*dw_a (total, incl. damping).
		CHECK_CLOSE_REL(J, moment*(wb1 - wb0), 1e-3, "damped J == I_b*dw_b", i);
		CHECK_CLOSE_REL(J, -moment*(wa1 - wa0), 1e-3, "damped J == -I_a*dw_a", i);

		// The damping contribution J - springTorque*dt must oppose the relative
		// angular velocity: its sign (impulse on b) equals sign(w_a - w_b).
		cpFloat dampImpulse = J - g_stepSpringImpulse;
		sumDampImpulse += dampImpulse;
		cpFloat wRel = wa0 - wb0;
		if(fabs(wRel) > 1e-4 && fabs(dampImpulse) > 1e-9){
			if(dampImpulse*wRel <= 0.0){
				g_failures++;
				printf("FAIL [damped damping-sign] step %d: dampImpulse=%.10g wRel=%.10g\n",
					i, (double)dampImpulse, (double)wRel);
			}
		}
		if(g_trace) fprintf(g_trace, "damped,%d,%.17g,%.17g,%.17g,%.17g\n", i,
			(double)cpBodyGetAngle(A), (double)cpBodyGetAngle(B),
			(double)wa1, (double)wb1);
	}

	// With damping != 0 the total must differ from the spring-only sum (the
	// damping term is actually included in the report).
	if(fabs(sumReported - g_sumSpringImpulse) < 1e-6){
		g_failures++;
		printf("FAIL [damped includes-damping]: total %.10g == spring-only %.10g\n",
			(double)sumReported, (double)g_sumSpringImpulse);
	}
	printf("damped: Sigma(reported)=%.10g Sigma(torque*dt)=%.10g Sigma(damp)=%.10g\n",
		(double)sumReported, (double)g_sumSpringImpulse, (double)sumDampImpulse);

	cpSpaceFree(space);
	cpConstraintFree(spring);
	cpBodyFree(A);
	cpBodyFree(B);
}

// Reference: cpDampedSpring impulse semantics (must remain unchanged).
// Same convention: total impulse of the step (spring + damping) applied to
// body b along n.
static cpVect g_springN;

static void
dampedSpringPreSolve(cpConstraint *constraint, cpSpace *space)
{
	cpBody *a = cpConstraintGetBodyA(constraint);
	cpBody *b = cpConstraintGetBodyB(constraint);
	cpVect delta = cpvsub(cpBodyGetPosition(b), cpBodyGetPosition(a));
	g_springN = cpvnormalize(delta);
}

static void
test_damped_spring_reference(void)
{
	cpSpace *space = cpSpaceNew();
	cpSpaceSetGravity(space, cpvzero);
	cpSpaceSetDamping(space, 1.0f);

	cpFloat mass = 2.0f;
	cpBody *A = cpSpaceAddBody(space, cpBodyNew(mass, 1.0f));
	cpBody *B = cpSpaceAddBody(space, cpBodyNew(mass, 1.0f));
	cpBodySetPosition(B, cpv(1.5f, 0.0f));

	cpConstraint *spring = cpDampedSpringNew(A, B, cpvzero, cpvzero, 1.0f, 100.0f, 5.0f);
	cpSpaceAddConstraint(space, spring);
	cpConstraintSetPreSolveFunc(spring, dampedSpringPreSolve);

	// preSolve has not run yet; seed the normal from the initial positions.
	g_springN = cpvnormalize(cpvsub(cpBodyGetPosition(B), cpBodyGetPosition(A)));

	int N = 60;
	for(int i = 0; i < N; i++){
		cpVect va0 = cpBodyGetVelocity(A);
		cpVect vb0 = cpBodyGetVelocity(B);
		cpVect n = g_springN; // normal used by the upcoming step (positions unchanged yet)

		cpSpaceStep(space, g_dt);
		cpFloat J = cpConstraintGetImpulse(spring);

		cpVect va1 = cpBodyGetVelocity(A);
		cpVect vb1 = cpBodyGetVelocity(B);

		CHECK_CLOSE_REL(J, mass*cpvdot(cpvsub(vb1, vb0), n), 1e-3, "spring J == m_b*dv_b.n", i);
		CHECK_CLOSE_REL(J, -mass*cpvdot(cpvsub(va1, va0), n), 1e-3, "spring J == -m_a*dv_a.n", i);
		if(g_trace) fprintf(g_trace, "spring,%d,%.17g,%.17g,%.17g,%.17g\n", i,
			(double)cpBodyGetPosition(A).x, (double)cpBodyGetPosition(B).x,
			(double)va1.x, (double)vb1.x);
	}
	printf("cpDampedSpring reference: impulse == total momentum change along n (incl. damping)\n");

	cpSpaceFree(space);
	cpConstraintFree(spring);
	cpBodyFree(A);
	cpBodyFree(B);
}

int
main(int argc, char **argv)
{
	if(argc > 1){
		g_trace = fopen(argv[1], "w");
		if(!g_trace){ perror("fopen"); return 2; }
	}

	g_dt = 1.0f/240.0f;

	test_pure_spring();
	test_damped_rotary();
	test_damped_spring_reference();

	if(g_trace) fclose(g_trace);

	if(g_failures){
		printf("RESULT: %d check(s) FAILED\n", g_failures);
		return 1;
	}
	printf("RESULT: all checks passed\n");
	return 0;
}
