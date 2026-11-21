// Acceptance tests for the sleeping component flood-fill fix.
//
// Exercises sleeping/waking over several deterministic contact/joint
// topologies (chain, pyramid with contact cycles, multi-contact-pair chain,
// joint ring) plus a regression scenario for the infinite loop in
// FloodFillComponent() that occurred when a sleeping component was activated
// while the space was locked (its stale-threaded arbiters were pushed into
// the contact graph a second time, creating a cyclic arbiter list).
//
// The program fails (exit code 1) on the first violated assertion and
// succeeds (exit code 0) only if every scenario completes within the step
// budget with the expected per-body sleep/wake behavior.
//
// A "dump" mode prints full trajectories so the output of the fixed library
// can be diffed bitwise against the unmodified baseline (both with sleeping
// enabled and disabled) to prove the fix does not change simulation
// semantics.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "chipmunk/chipmunk.h"

static int failures = 0;
static const char *current_scenario = "?";

#define CHECK(cond, msg) do { \
	if(!(cond)){ \
		fprintf(stderr, "FAIL [%s] step %d: %s (%s:%d)\n", current_scenario, step, msg, __FILE__, __LINE__); \
		exit(1); \
	} \
} while(0)

#define STEPS 2400
#define SETTLE_STEPS 600
#define DT (1.0f/60.0f)
#define SLEEP_THRESHOLD 0.5f

typedef struct {
	cpSpace *space;
	cpBody **comp;     // bodies of the tested component
	int compCount;
	cpBody **control;  // far-away control component, must stay asleep
	int controlCount;
	cpBody *ball;      // disturber
} Scene;

static cpShape *lastShape = NULL;

static cpBody *
addBox(cpSpace *space, cpFloat x, cpFloat y, cpFloat w, cpFloat h)
{
	cpBody *body = cpSpaceAddBody(space, cpBodyNew(1.0f, cpMomentForBox(1.0f, w, h)));
	cpBodySetPosition(body, cpv(x, y));
	cpShape *shape = cpSpaceAddShape(space, cpBoxShapeNew(body, w, h, 0.0f));
	cpShapeSetFriction(shape, 0.7f);
	lastShape = shape;
	return body;
}

static cpBody *
addBall(cpSpace *space, cpFloat x, cpFloat y, cpFloat r)
{
	cpBody *body = cpSpaceAddBody(space, cpBodyNew(1.0f, cpMomentForCircle(1.0f, 0.0f, r, cpvzero)));
	cpBodySetPosition(body, cpv(x, y));
	cpShape *shape = cpSpaceAddShape(space, cpCircleShapeNew(body, r, cpvzero));
	cpShapeSetFriction(shape, 0.7f);
	lastShape = shape;
	return body;
}

static cpSpace *
makeSpace(void)
{
	cpSpace *space = cpSpaceNew();
	cpSpaceSetGravity(space, cpv(0.0f, -100.0f));
	cpSpaceSetSleepTimeThreshold(space, SLEEP_THRESHOLD);

	cpShape *ground = cpSegmentShapeNew(cpSpaceGetStaticBody(space), cpv(-2000.0f, 0.0f), cpv(2000.0f, 0.0f), 0.0f);
	cpShapeSetFriction(ground, 1.0f);
	cpSpaceAddShape(space, ground);
	return space;
}

static void
addControl(Scene *scene)
{
	// A separate component far away that must fall asleep and never wake up.
	scene->control = (cpBody **)calloc(2, sizeof(cpBody *));
	scene->control[0] = addBox(scene->space, 500.0f, 10.0f, 20.0f, 20.0f);
	scene->control[1] = addBox(scene->space, 519.0f, 10.0f, 20.0f, 20.0f);
	scene->controlCount = 2;
}

static int
allSleeping(cpBody **bodies, int count)
{
	for(int i = 0; i < count; i++) if(!cpBodyIsSleeping(bodies[i])) return 0;
	return 1;
}

static int
allAwake(cpBody **bodies, int count)
{
	for(int i = 0; i < count; i++) if(cpBodyIsSleeping(bodies[i])) return 0;
	return 1;
}

static int
anyAwake(cpBody **bodies, int count)
{
	for(int i = 0; i < count; i++) if(!cpBodyIsSleeping(bodies[i])) return 1;
	return 0;
}

// Common driver for the "settle -> disturb by falling ball -> wake -> re-sleep" cycle.
// dropX/dropY: where the disturber ball is teleported to at step SETTLE_STEPS.
static void
runBallDropScenario(Scene *scene, cpFloat dropX, cpFloat dropY)
{
	int step = 0;
	int wakeStep = -1;

	for(step = 0; step < STEPS; step++){
		if(step == SETTLE_STEPS){
			// The whole component must be fast asleep by now.
			CHECK(allSleeping(scene->comp, scene->compCount), "component did not fall asleep while resting");
			CHECK(allSleeping(scene->control, scene->controlCount), "control component did not fall asleep");
			CHECK(allSleeping(&scene->ball, 1), "disturber ball did not fall asleep");

			// Drop the disturber ball onto the component.
			cpBodySetPosition(scene->ball, cpv(dropX, dropY));
			cpBodySetVelocity(scene->ball, cpvzero);
			cpBodySetAngle(scene->ball, 0.0f);
			cpBodyActivate(scene->ball);
		}

		cpSpaceStep(scene->space, DT);

		if(step > SETTLE_STEPS && wakeStep < 0 && anyAwake(scene->comp, scene->compCount)){
			// The first step any body of the component is awake, the whole
			// component must be awake: components wake up atomically.
			wakeStep = step;
			CHECK(allAwake(scene->comp, scene->compCount), "component did not wake up as a whole");
			CHECK(allSleeping(scene->control, scene->controlCount), "control component woke up spuriously");
		}
	}

	CHECK(wakeStep > SETTLE_STEPS, "component never woke up after the ball impact");
	// Everything must have settled back to sleep.
	CHECK(allSleeping(scene->comp, scene->compCount), "component did not fall back asleep");
	CHECK(allSleeping(&scene->ball, 1), "disturber ball did not fall back asleep");
	CHECK(allSleeping(scene->control, scene->controlCount), "control component did not stay asleep");
}

static void ShapeFreeWrap(cpSpace *space, cpShape *shape, void *unused){
	(void)unused; cpSpaceRemoveShape(space, shape); cpShapeFree(shape);
}
static void PostShapeFree(cpShape *shape, cpSpace *space){
	cpSpaceAddPostStepCallback(space, (cpPostStepFunc)ShapeFreeWrap, shape, NULL);
}
static void ConstraintFreeWrap(cpSpace *space, cpConstraint *constraint, void *unused){
	(void)unused; cpSpaceRemoveConstraint(space, constraint); cpConstraintFree(constraint);
}
static void PostConstraintFree(cpConstraint *constraint, cpSpace *space){
	cpSpaceAddPostStepCallback(space, (cpPostStepFunc)ConstraintFreeWrap, constraint, NULL);
}
static void BodyFreeWrap(cpSpace *space, cpBody *body, void *unused){
	(void)unused; cpSpaceRemoveBody(space, body); cpBodyFree(body);
}
static void PostBodyFree(cpBody *body, cpSpace *space){
	cpSpaceAddPostStepCallback(space, (cpPostStepFunc)BodyFreeWrap, body, NULL);
}

// Free everything owned by the caller (Chipmunk does not free bodies,
// shapes or constraints in cpSpaceFree()). Same pattern as the demos.
static void
freeSpaceChildren(cpSpace *space)
{
	cpSpaceEachShape(space, (cpSpaceShapeIteratorFunc)PostShapeFree, space);
	cpSpaceEachConstraint(space, (cpSpaceConstraintIteratorFunc)PostConstraintFree, space);
	cpSpaceEachBody(space, (cpSpaceBodyIteratorFunc)PostBodyFree, space);
}

static void
freeScene(Scene *scene)
{
	free(scene->comp);
	free(scene->control);
	freeSpaceChildren(scene->space);
	cpSpaceFree(scene->space);
}

// Scenario 1: chain of boxes in a row, each touching its neighbors (line topology).
static void
scenarioChain(void)
{
	current_scenario = "chain";
	Scene scene = {0};
	scene.space = makeSpace();

	int n = 6;
	scene.comp = (cpBody **)calloc(n, sizeof(cpBody *));
	for(int i = 0; i < n; i++) scene.comp[i] = addBox(scene.space, 19.0f*i, 10.0f, 20.0f, 20.0f);
	scene.compCount = n;

	addControl(&scene);
	scene.ball = addBall(scene.space, -500.0f, 5.0f, 5.0f);

	runBallDropScenario(&scene, 2.5f*19.0f, 200.0f);
	freeScene(&scene);
	printf("PASS [%s]\n", current_scenario);
}

// Scenario 2: pyramid of boxes; the contact graph contains cycles
// (each upper box touches two lower boxes that also touch each other).
static void
scenarioPyramid(void)
{
	current_scenario = "pyramid";
	Scene scene = {0};
	scene.space = makeSpace();

	int n = 0;
	scene.comp = (cpBody **)calloc(6, sizeof(cpBody *));
	scene.comp[n++] = addBox(scene.space, -19.0f, 10.0f, 20.0f, 20.0f);
	scene.comp[n++] = addBox(scene.space,   0.0f, 10.0f, 20.0f, 20.0f);
	scene.comp[n++] = addBox(scene.space,  19.0f, 10.0f, 20.0f, 20.0f);
	scene.comp[n++] = addBox(scene.space,  -9.5f, 30.0f, 20.0f, 20.0f);
	scene.comp[n++] = addBox(scene.space,   9.5f, 30.0f, 20.0f, 20.0f);
	scene.comp[n++] = addBox(scene.space,   0.0f, 50.0f, 20.0f, 20.0f);
	scene.compCount = n;

	addControl(&scene);
	scene.ball = addBall(scene.space, -500.0f, 5.0f, 5.0f);

	runBallDropScenario(&scene, 0.0f, 250.0f);
	freeScene(&scene);
	printf("PASS [%s]\n", current_scenario);
}

// Scenario 3: chain of bodies where each neighbor pair shares TWO contact
// pairs (two circle shapes per body, both overlapping the neighbor's).
static void
scenarioMultiContactPair(void)
{
	current_scenario = "multi-contact-pair";
	Scene scene = {0};
	scene.space = makeSpace();

	int n = 5;
	scene.comp = (cpBody **)calloc(n, sizeof(cpBody *));
	for(int i = 0; i < n; i++){
		cpBody *body = cpSpaceAddBody(scene.space, cpBodyNew(2.0f, cpMomentForBox(2.0f, 24.0f, 30.0f)));
		cpBodySetPosition(body, cpv(15.0f*i, 16.0f));
		cpShape *s1 = cpSpaceAddShape(scene.space, cpCircleShapeNew(body, 8.0f, cpv(0.0f,  8.0f)));
		cpShape *s2 = cpSpaceAddShape(scene.space, cpCircleShapeNew(body, 8.0f, cpv(0.0f, -8.0f)));
		cpShapeSetFriction(s1, 0.7f);
		cpShapeSetFriction(s2, 0.7f);
		scene.comp[i] = body;
	}
	scene.compCount = n;

	addControl(&scene);
	scene.ball = addBall(scene.space, -500.0f, 5.0f, 5.0f);

	runBallDropScenario(&scene, 2.0f*15.0f, 200.0f);
	freeScene(&scene);
	printf("PASS [%s]\n", current_scenario);
}

// Scenario 4: boxes resting on the ground WITHOUT touching each other,
// connected by pin joints into a ring (cycle in the constraint graph).
// Woken by an impulse instead of a collision.
static void
scenarioJointRing(void)
{
	current_scenario = "joint-ring";
	Scene scene = {0};
	scene.space = makeSpace();

	int n = 6;
	scene.comp = (cpBody **)calloc(n, sizeof(cpBody *));
	for(int i = 0; i < n; i++) scene.comp[i] = addBox(scene.space, 21.0f*i, 10.0f, 20.0f, 20.0f);
	scene.compCount = n;

	for(int i = 0; i < n; i++){
		cpBody *a = scene.comp[i];
		cpBody *b = scene.comp[(i + 1)%n];
		cpConstraint *joint = cpPinJointNew(a, b, cpvzero, cpvzero);
		cpPinJointSetDist(joint, 21.0f*((i + 1)%n > i ? 1 : n - 1));
		cpSpaceAddConstraint(scene.space, joint);
	}

	addControl(&scene);

	int step = 0;
	for(step = 0; step < STEPS; step++){
		if(step == SETTLE_STEPS){
			CHECK(allSleeping(scene.comp, scene.compCount), "joint ring did not fall asleep");
			CHECK(allSleeping(scene.control, scene.controlCount), "control component did not fall asleep");

			// Wake one body with an impulse: the whole jointed component must wake.
			cpBodyApplyImpulseAtWorldPoint(scene.comp[0], cpv(0.0f, 500.0f), cpBodyGetPosition(scene.comp[0]));
			CHECK(allAwake(scene.comp, scene.compCount), "joint ring did not wake up as a whole");
			CHECK(allSleeping(scene.control, scene.controlCount), "control component woke up spuriously");
		}

		cpSpaceStep(scene.space, DT);
	}

	CHECK(allSleeping(scene.comp, scene.compCount), "joint ring did not fall back asleep");
	CHECK(allSleeping(scene.control, scene.controlCount), "control component did not stay asleep");
	freeScene(&scene);
	printf("PASS [%s]\n", current_scenario);
}

// Scenario 5 (regression): a sleeping component is activated from a collision
// callback while the space is locked. The restored arbiters of the component
// are still threaded in the contact graph from before the component slept;
// re-threading them used to create a cyclic arbiter list and hang
// FloodFillComponent() (infinite loop in CP_BODY_FOREACH_ARBITER).
static cpBody *regressionSleeper = NULL;
static int regressionActivations = 0;

static cpBool
regressionPreSolve(cpArbiter *arb, cpSpace *space, void *data)
{
	(void)arb; (void)space; (void)data;
	if(regressionSleeper && cpBodyIsSleeping(regressionSleeper) && regressionActivations == 0){
		// Activate the sleeping body while the space is locked.
		// The activation is deferred and performed at cpSpaceUnlock(),
		// right before the contact graph is rebuilt.
		cpBodyActivate(regressionSleeper);
		regressionActivations++;
	}
	return cpTrue;
}

static void
scenarioLockedActivation(void)
{
	current_scenario = "locked-activation";
	regressionSleeper = NULL;
	regressionActivations = 0;

	cpSpace *space = makeSpace();

	cpBody *A = addBox(space, 0.0f, 10.0f, 20.0f, 20.0f);
	cpBody *B = addBox(space, 19.0f, 10.0f, 20.0f, 20.0f);
	cpBody *comp[2] = {A, B};

	// Trigger ball far away; its collisions with the ground (type 7 vs 0)
	// invoke the callback that activates the sleeper while locked.
	cpBody *ball = addBall(space, -500.0f, 5.0f, 5.0f);
	cpShapeSetCollisionType(lastShape, 7);

	cpCollisionHandler *handler = cpSpaceAddCollisionHandler(space, 7, 0);
	handler->preSolveFunc = regressionPreSolve;

	int step = 0;
	int wokeStep = -1;

	for(step = 0; step < 2000; step++){
		if(step == SETTLE_STEPS){
			CHECK(allSleeping(comp, 2), "component did not fall asleep");
			regressionSleeper = A;
			cpBodySetPosition(ball, cpv(300.0f, 50.0f));
			cpBodySetVelocity(ball, cpvzero);
			cpBodyActivate(ball);
		}

		cpSpaceStep(space, DT);

		if(wokeStep < 0 && regressionActivations > 0){
			// The step the callback fired on must have woken the whole component.
			wokeStep = step;
			CHECK(allAwake(comp, 2), "component did not wake up as a whole after locked activation");
		}
	}

	CHECK(regressionActivations == 1, "locked activation callback never fired");
	CHECK(allSleeping(comp, 2), "component did not fall back asleep");

	freeSpaceChildren(space);
	cpSpaceFree(space);
	printf("PASS [%s]\n", current_scenario);
}

// Scenario 6: sleeping disabled. Bodies must never go to sleep and the
// simulation must run to completion.
static void
scenarioNoSleep(void)
{
	current_scenario = "no-sleep";
	cpSpace *space = cpSpaceNew();
	cpSpaceSetGravity(space, cpv(0.0f, -100.0f));
	// sleepTimeThreshold stays at INFINITY: sleeping disabled.

	cpShape *ground = cpSegmentShapeNew(cpSpaceGetStaticBody(space), cpv(-2000.0f, 0.0f), cpv(2000.0f, 0.0f), 0.0f);
	cpShapeSetFriction(ground, 1.0f);
	cpSpaceAddShape(space, ground);

	int n = 6;
	cpBody **bodies = (cpBody **)calloc(n, sizeof(cpBody *));
	for(int i = 0; i < n; i++) bodies[i] = addBox(space, 19.0f*i, 10.0f, 20.0f, 20.0f);

	int step = 0;
	for(step = 0; step < STEPS; step++){
		cpSpaceStep(space, DT);
		for(int i = 0; i < n; i++){
			CHECK(!cpBodyIsSleeping(bodies[i]), "body fell asleep with sleeping disabled");
		}
	}

	free(bodies);
	freeSpaceChildren(space);
	cpSpaceFree(space);
	printf("PASS [%s]\n", current_scenario);
}

// Dump mode: print the full trajectory of the chain scenario (with and
// without sleeping) so fixed vs. baseline library output can be diffed to
// prove the fix does not change simulation semantics.
static void
dumpTrajectory(int sleepEnabled)
{
	cpSpace *space = cpSpaceNew();
	cpSpaceSetGravity(space, cpv(0.0f, -100.0f));
	if(sleepEnabled) cpSpaceSetSleepTimeThreshold(space, SLEEP_THRESHOLD);

	cpShape *ground = cpSegmentShapeNew(cpSpaceGetStaticBody(space), cpv(-2000.0f, 0.0f), cpv(2000.0f, 0.0f), 0.0f);
	cpShapeSetFriction(ground, 1.0f);
	cpSpaceAddShape(space, ground);

	int n = 6;
	cpBody **bodies = (cpBody **)calloc(n, sizeof(cpBody *));
	for(int i = 0; i < n; i++) bodies[i] = addBox(space, 19.0f*i, 10.0f, 20.0f, 20.0f);
	cpBody *ball = addBall(space, -500.0f, 5.0f, 5.0f);

	for(int step = 0; step < STEPS; step++){
		if(step == SETTLE_STEPS){
			cpBodySetPosition(ball, cpv(2.5f*19.0f, 200.0f));
			cpBodySetVelocity(ball, cpvzero);
			cpBodyActivate(ball);
		}
		cpSpaceStep(space, DT);
		for(int i = 0; i < n; i++){
			cpVect p = cpBodyGetPosition(bodies[i]);
			cpVect v = cpBodyGetVelocity(bodies[i]);
			printf("%d %d %.9g %.9g %.9g %.9g %d\n", step, i, p.x, p.y, v.x, v.y, cpBodyIsSleeping(bodies[i]));
		}
		{
			cpVect p = cpBodyGetPosition(ball);
			printf("%d b %.9g %.9g %d\n", step, p.x, p.y, cpBodyIsSleeping(ball));
		}
	}

	free(bodies);
	freeSpaceChildren(space);
	cpSpaceFree(space);
}

int
main(int argc, char **argv)
{
	if(argc > 1 && strcmp(argv[1], "dump") == 0){
		dumpTrajectory(1);
		return 0;
	}
	if(argc > 1 && strcmp(argv[1], "dump-nosleep") == 0){
		dumpTrajectory(0);
		return 0;
	}

	scenarioChain();
	scenarioPyramid();
	scenarioMultiContactPair();
	scenarioJointRing();
	scenarioLockedActivation();
	scenarioNoSleep();

	printf("ALL SLEEP COMPONENT TESTS PASSED\n");
	return failures ? 1 : 0;
}
