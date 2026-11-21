// Hang repro: shapes with degenerate/astronomic bbs must not hang cpSpaceStep.
// With the bug, each of these cases loops (nearly) forever in the spatial hash.
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "chipmunk/chipmunk.h"
#include "chipmunk/chipmunk_private.h"

static double now(void){
	struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec*1e-9;
}

static void pq_cb(cpShape *s, cpVect p, cpFloat d, cpVect g, void *data){
	(void)s;(void)p;(void)d;(void)g;(void)data;
}

int main(void){
	// Case 1: body teleported to an astronomic coordinate (bb cell coords
	// near INT_MAX -> loop counter overflow / billions of iterations).
	{
		cpSpace *space = cpSpaceNew();
		cpSpaceUseSpatialHash(space, 5.0, 500);
		cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, 1));
		cpShape *shape = cpSpaceAddShape(space, cpCircleShapeNew(body, 2, cpvzero));
		cpBodySetPosition(body, cpv(1e10, -1e10));
		double t0 = now();
		for(int i = 0; i < 10; i++) cpSpaceStep(space, 1.0/60.0);
		printf("case1 (huge coords): %.3fs\n", now() - t0);
		cpSpaceFree(space);
		(void)shape;
	}
	// Case 2: NaN position.
	{
		cpSpace *space = cpSpaceNew();
		cpSpaceUseSpatialHash(space, 5.0, 500);
		cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, 1));
		cpSpaceAddShape(space, cpCircleShapeNew(body, 2, cpvzero));
		body->p = cpv(NAN, NAN); // bypass cpBodySetPosition's debug assert
		double t0 = now();
		for(int i = 0; i < 10; i++) cpSpaceStep(space, 1.0/60.0);
		printf("case2 (NaN coords): %.3fs\n", now() - t0);
		cpSpaceFree(space);
	}
	// Case 3: +/-Inf position.
	{
		cpSpace *space = cpSpaceNew();
		cpSpaceUseSpatialHash(space, 5.0, 500);
		cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, 1));
		cpSpaceAddShape(space, cpCircleShapeNew(body, 2, cpvzero));
		body->p = cpv(INFINITY, -INFINITY); // bypass debug assert
		double t0 = now();
		for(int i = 0; i < 10; i++) cpSpaceStep(space, 1.0/60.0);
		printf("case3 (Inf coords): %.3fs\n", now() - t0);
		cpSpaceFree(space);
	}
	// Case 4: gigantic shape (span >> numcells).
	{
		cpSpace *space = cpSpaceNew();
		cpSpaceUseSpatialHash(space, 5.0, 500);
		cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, INFINITY));
		cpSpaceAddShape(space, cpBoxShapeNew(body, 1e8, 1e8, 0));
		double t0 = now();
		for(int i = 0; i < 10; i++) cpSpaceStep(space, 1.0/60.0);
		printf("case4 (giant shape): %.3fs\n", now() - t0);
		cpSpaceFree(space);
	}
	// Case 5: huge-bb point query.
	{
		cpSpace *space = cpSpaceNew();
		cpSpaceUseSpatialHash(space, 5.0, 500);
		cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, 1));
		cpSpaceAddShape(space, cpCircleShapeNew(body, 2, cpvzero));
		double t0 = now();
		for(int i = 0; i < 10; i++)
			cpSpacePointQuery(space, cpv(1e10, 1e10), 1e9, CP_SHAPE_FILTER_ALL, pq_cb, NULL);
		printf("case5 (huge query): %.3fs\n", now() - t0);
		cpSpaceFree(space);
	}
	printf("DONE\n");
	return 0;
}
