// Mega fuzzer: exercises every spatial-hash mutation path.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "chipmunk/chipmunk.h"
#include "chipmunk/chipmunk_private.h"

static unsigned long long rng_state;
static unsigned int rnd(void){
	rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
	return (unsigned int)(rng_state >> 32);
}
static cpFloat frand(cpFloat lo, cpFloat hi){
	return lo + (hi - lo) * (cpFloat)(rnd() % 100000) / 100000.0f;
}

#define MAXSHAPES 150
static cpSpace *g_space;
typedef struct { cpBody *body; cpShape *shape; int live; } Ent;
static Ent ents[MAXSHAPES];

#define CHECK() do{}while(0)

static void add_shape(int slot){
	cpFloat m = frand(0.5, 3.0);
	cpBody *body;
	cpBodyType type = (rnd()%10 == 0) ? CP_BODY_TYPE_KINEMATIC : CP_BODY_TYPE_DYNAMIC;
	body = cpBodyNew(m, cpMomentForBox(m, 4, 4));
	cpBodySetType(body, type);
	cpBodySetPosition(body, cpv(frand(-100, 100), frand(-100, 100)));
	cpSpaceAddBody(g_space, body);
	cpShape *shape = (rnd()%2) ? cpBoxShapeNew(body, frand(2,12), frand(2,12), 0.0)
	                           : cpCircleShapeNew(body, frand(1,6), cpvzero);
	cpShapeSetFriction(shape, 0.7);
	cpShapeSetDensity(shape, 1.0);
	cpSpaceAddShape(g_space, shape);
	ents[slot].body = body; ents[slot].shape = shape; ents[slot].live = 1;
}
static void remove_shape(int slot){
	cpBody *body = ents[slot].body;
	cpSpaceRemoveShape(g_space, ents[slot].shape);
	cpShapeFree(ents[slot].shape);
	cpSpaceRemoveBody(g_space, body);
	cpBodyFree(body);
	ents[slot].live = 0; ents[slot].body = NULL; ents[slot].shape = NULL;
}
static void pq_cb(cpShape *s, cpVect p, cpFloat d, cpVect g, void *data){(void)s;(void)p;(void)d;(void)g;(void)data;}
static void sq_cb(cpShape *s, cpContactPointSet *set, void *data){(void)s;(void)set;(void)data;}
static void seg_cb(cpShape *s, cpVect p, cpVect n, cpFloat a, void *d){(void)s;(void)p;(void)n;(void)a;(void)d;}
static void bb_cb(cpShape *s, void *d){(void)s;(void)d;}

int main(int argc, char **argv){
	int N = argc > 1 ? atoi(argv[1]) : 2000;
	rng_state = argc > 2 ? strtoull(argv[2], NULL, 10) : 1;

	g_space = cpSpaceNew();
	cpSpaceSetGravity(g_space, cpv(0, -50));
	cpSpaceSetSleepTimeThreshold(g_space, 0.5f);
	if(!getenv("NOHASH")) cpSpaceUseSpatialHash(g_space, (cpFloat)(1 + rnd()%8), 50 + rnd()%1000);

	cpBody *stat = cpSpaceGetStaticBody(g_space);
	cpShape *segs[4];
	for(int i = 0; i < 4; i++){
		cpShape *seg = cpSegmentShapeNew(stat, cpv(-200+i*100, -60), cpv(-100+i*100, -60), 2);
		cpShapeSetFriction(seg, 1.0);
		cpSpaceAddShape(g_space, seg);
		segs[i] = seg;
	}

	for(int step = 0; step < N; step++){
		int ops = 1 + rnd() % 6;
		for(int k = 0; k < ops; k++){
			int slot = rnd() % MAXSHAPES;
			switch(rnd() % 14){
			case 0: case 1: case 2: case 3:
				if(!ents[slot].live) add_shape(slot);
				break;
			case 4: case 5:
				if(ents[slot].live) remove_shape(slot);
				break;
			case 6: case 7:
				if(ents[slot].live){
					cpBodySetPosition(ents[slot].body, cpv(frand(-120,120), frand(-60,120)));
					cpBodyActivate(ents[slot].body);
				}
				break;
			case 8:
				if(ents[slot].live) cpSpaceReindexShape(g_space, ents[slot].shape);
				break;
			case 9:
				if(ents[slot].live) cpSpaceReindexShapesForBody(g_space, ents[slot].body);
				break;
			case 10:
				cpSpaceReindexStatic(g_space);
				break;
			case 11:
				if(ents[slot].live) cpBodyActivate(ents[slot].body);
				break;
			case 12:
				if(ents[slot].live){
					cpVect p = cpBodyGetPosition(ents[slot].body);
					cpBodySetPosition(ents[slot].body, cpv(p.x + frand(-3,3), p.y + frand(-3,3)));
					cpBodyActivate(ents[slot].body);
				}
				break;
			case 13:
				if(ents[slot].live) cpBodyActivate(ents[slot].body);
				break;
			}
			CHECK();
		}
		cpSpaceStep(g_space, 1.0/60.0);
		CHECK();
		cpSpacePointQuery(g_space, cpv(frand(-150,150), frand(-80,150)), 20.0, CP_SHAPE_FILTER_ALL, pq_cb, NULL);
		cpSpaceSegmentQuery(g_space, cpv(frand(-150,150), frand(-80,150)), cpv(frand(-150,150), frand(-80,150)), 1.0, CP_SHAPE_FILTER_ALL, seg_cb, NULL);
		cpSpaceBBQuery(g_space, cpBBNew(frand(-150,0), frand(-80,0), frand(0,150), frand(0,150)), CP_SHAPE_FILTER_ALL, bb_cb, NULL);
		if(ents[step % MAXSHAPES].live)
			cpSpaceShapeQuery(g_space, ents[step % MAXSHAPES].shape, sq_cb, NULL);
		CHECK();
		if(step % 500 == 0){ printf("step %d\n", step); fflush(stdout); }
	}
	for(int i = 0; i < MAXSHAPES; i++) if(ents[i].live) remove_shape(i);
	for(int i = 0; i < 4; i++){ cpSpaceRemoveShape(g_space, segs[i]); cpShapeFree(segs[i]); }
	cpSpaceFree(g_space);
	printf("DONE %d\n", N);
	return 0;
}
