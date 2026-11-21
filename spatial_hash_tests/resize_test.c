// cpSpaceHashResize must keep the index consistent: shapes (especially
// static ones, which cpSpaceStep never rehashes) must remain queryable.
#include <stdio.h>
#include "chipmunk/chipmunk.h"
#include "chipmunk/chipmunk_private.h"

static int hits;
static void pq_cb(cpShape *s, cpVect p, cpFloat d, cpVect g, void *data){
	(void)p;(void)d;(void)g;(void)data;
	if(s) hits++;
}

int main(void){
	cpSpace *space = cpSpaceNew();
	cpSpaceUseSpatialHash(space, 5.0, 500);

	cpBody *stat = cpSpaceGetStaticBody(space);
	cpShape *seg = cpSegmentShapeNew(stat, cpv(-100, 0), cpv(100, 0), 1);
	cpSpaceAddShape(space, seg);

	cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, 1));
	cpBodySetPosition(body, cpv(0, 10));
	cpShape *ball = cpSpaceAddShape(space, cpCircleShapeNew(body, 5, cpvzero));

	hits = 0;
	cpSpacePointQuery(space, cpv(0, 0), 2.0, CP_SHAPE_FILTER_ALL, pq_cb, NULL);
	printf("before resize: hits=%d (expect 1)\n", hits);
	if(hits != 1) return 1;

	cpSpaceHashResize((cpSpaceHash *)space->staticShapes, 10.0, 1000);
	cpSpaceHashResize((cpSpaceHash *)space->dynamicShapes, 10.0, 1000);

	hits = 0;
	cpSpacePointQuery(space, cpv(0, 0), 2.0, CP_SHAPE_FILTER_ALL, pq_cb, NULL);
	printf("after resize (static): hits=%d (expect 1)\n", hits);
	if(hits != 1) return 2;

	for(int i = 0; i < 5; i++) cpSpaceStep(space, 1.0/60.0);

	hits = 0;
	cpSpacePointQuery(space, cpv(0, 0), 2.0, CP_SHAPE_FILTER_ALL, pq_cb, NULL);
	int static_ok = hits;
	hits = 0;
	cpVect p = cpBodyGetPosition(body);
	cpSpacePointQuery(space, p, 6.0, CP_SHAPE_FILTER_ALL, pq_cb, NULL);
	printf("after steps: static_hits=%d (expect 1) dyn_hits=%d (expect >=1)\n", static_ok, hits);
	if(static_ok != 1 || hits < 1) return 3;

	cpSpaceRemoveShape(space, ball); cpShapeFree(ball);
	cpSpaceRemoveBody(space, body); cpBodyFree(body);
	cpSpaceRemoveShape(space, seg); cpShapeFree(seg);
	cpSpaceFree(space);
	printf("RESIZE_OK\n");
	return 0;
}
