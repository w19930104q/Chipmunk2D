// Differential test: identical random op sequence applied to a spatial-hash
// space and a BB-tree space; point/shape query results must match exactly.
// All randomness is pre-generated into an op list so both worlds see
// bit-identical inputs. Shapes are sensors with zero gravity so physics
// cannot diverge the layouts.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "chipmunk/chipmunk.h"

#define MAXSHAPES 120
#define MAXQUERIES 8
#define MAXOPS 4

static unsigned long long rng_state;
static unsigned int rnd(void){
	rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
	return (unsigned int)(rng_state >> 32);
}
static cpFloat frand(cpFloat lo, cpFloat hi){
	return lo + (hi - lo) * (cpFloat)(rnd() % 100000) / 100000.0f;
}

typedef struct { cpBody *body; cpShape *shape; int live; int id; } Ent;
typedef struct { cpSpace *space; Ent ents[MAXSHAPES]; int next_id; cpShape *segs[4]; } World;

typedef struct {
	int slot, action;
	cpFloat x, y;      // teleport target / add position / jitter
	cpFloat m, w, h;   // shape params for add
	int circle;
} Op;

static void add_shape(World *w, int slot, const Op *op){
	cpBody *body = cpBodyNew(op->m, cpMomentForBox(op->m, 4, 4));
	cpBodySetPosition(body, cpv(op->x, op->y));
	cpSpaceAddBody(w->space, body);
	cpShape *shape = op->circle
		? cpCircleShapeNew(body, op->w, cpvzero)
		: cpBoxShapeNew(body, op->w, op->h, 0.0);
	cpShapeSetFriction(shape, 0.7);
	cpShapeSetSensor(shape, cpTrue);
	int id = w->next_id++;
	cpShapeSetUserData(shape, (cpDataPointer)(intptr_t)id);
	cpSpaceAddShape(w->space, shape);
	w->ents[slot].body = body; w->ents[slot].shape = shape; w->ents[slot].live = 1; w->ents[slot].id = id;
}
static void remove_shape(World *w, int slot){
	cpSpaceRemoveShape(w->space, w->ents[slot].shape);
	cpShapeFree(w->ents[slot].shape);
	cpSpaceRemoveBody(w->space, w->ents[slot].body);
	cpBodyFree(w->ents[slot].body);
	w->ents[slot].live = 0; w->ents[slot].body = NULL; w->ents[slot].shape = NULL;
}

static int ids[4096];
static int nids;
static void pq_cb(cpShape *shape, cpVect p, cpFloat d, cpVect g, void *data){
	(void)p;(void)d;(void)g;(void)data;
	if(nids < 4096) ids[nids++] = (int)(intptr_t)cpShapeGetUserData(shape);
}
static void sq_cb(cpShape *shape, cpContactPointSet *set, void *data){
	(void)set;(void)data;
	if(nids < 4096) ids[nids++] = (int)(intptr_t)cpShapeGetUserData(shape);
}
static int int_cmp(const void *a, const void *b){ return *(const int*)a - *(const int*)b; }

static int collect_point(World *w, cpFloat x, cpFloat y, int *out){
	nids = 0;
	cpSpacePointQuery(w->space, cpv(x, y), 15.0, CP_SHAPE_FILTER_ALL, pq_cb, NULL);
	memcpy(out, ids, nids*sizeof(int));
	qsort(out, nids, sizeof(int), int_cmp);
	return nids;
}
static int collect_shape(World *w, cpShape *shape, int *out){
	nids = 0;
	cpSpaceShapeQuery(w->space, shape, sq_cb, NULL);
	memcpy(out, ids, nids*sizeof(int));
	qsort(out, nids, sizeof(int), int_cmp);
	return nids;
}

int main(int argc, char **argv){
	int N = argc > 1 ? atoi(argv[1]) : 2000;
	rng_state = argc > 2 ? strtoull(argv[2], NULL, 10) : 1;

	World wa, wb;
	memset(&wa, 0, sizeof(wa)); memset(&wb, 0, sizeof(wb));

	wa.space = cpSpaceNew();
	cpSpaceSetSleepTimeThreshold(wa.space, 0.5f);
	cpSpaceUseSpatialHash(wa.space, 5.0, 500);

	wb.space = cpSpaceNew();
	cpSpaceSetSleepTimeThreshold(wb.space, 0.5f);

	World *worlds[2] = {&wa, &wb};
	for(int wi = 0; wi < 2; wi++){
		cpBody *stat = cpSpaceGetStaticBody(worlds[wi]->space);
		for(int i = 0; i < 4; i++){
			cpShape *seg = cpSegmentShapeNew(stat, cpv(-200+i*100, -60), cpv(-100+i*100, -60), 2);
			cpShapeSetFriction(seg, 1.0);
			cpShapeSetUserData(seg, (cpDataPointer)(intptr_t)(1000000+i));
			cpSpaceAddShape(worlds[wi]->space, seg);
			worlds[wi]->segs[i] = seg;
		}
	}

	static int ida[4096], idb[4096];
	static Op ops[MAXOPS];

	for(int step = 0; step < N; step++){
		int nops = 1 + rnd() % MAXOPS;
		for(int k = 0; k < nops; k++){
			Op *op = &ops[k];
			op->slot = rnd() % MAXSHAPES;
			op->action = rnd() % 10;
			op->x = frand(-120, 120);
			op->y = frand(-60, 120);
			op->m = frand(0.5, 3.0);
			op->circle = rnd() % 2;
			op->w = frand(1, 6);
			op->h = frand(2, 12);
		}

		for(int wi = 0; wi < 2; wi++){
			World *w = worlds[wi];
			for(int k = 0; k < nops; k++){
				Op *op = &ops[k];
				int slot = op->slot;
				switch(op->action){
				case 0: case 1: case 2:
					if(!w->ents[slot].live) add_shape(w, slot, op);
					break;
				case 3: case 4:
					if(w->ents[slot].live) remove_shape(w, slot);
					break;
				case 5: case 6: case 7:
					if(w->ents[slot].live){
						cpBodySetPosition(w->ents[slot].body, cpv(op->x, op->y));
						cpBodyActivate(w->ents[slot].body);
						if(k % 3 == 0) cpSpaceReindexShape(w->space, w->ents[slot].shape);
					}
					break;
				case 8:
					if(w->ents[slot].live){
						cpVect p = cpBodyGetPosition(w->ents[slot].body);
						cpBodySetPosition(w->ents[slot].body, cpv(p.x + op->x/20.0, p.y + op->y/20.0));
						cpBodyActivate(w->ents[slot].body);
					}
					break;
				case 9:
					if(w->ents[slot].live) cpBodyActivate(w->ents[slot].body);
					break;
				}
			}
			cpSpaceStep(w->space, 1.0/60.0);
		}

		for(int q = 0; q < MAXQUERIES; q++){
			cpFloat qx = frand(-150, 150);
			cpFloat qy = frand(-80, 150);
			int na = collect_point(&wa, qx, qy, ida);
			int nb = collect_point(&wb, qx, qy, idb);
			if(na != nb || memcmp(ida, idb, na*sizeof(int))){
				printf("POINT MISMATCH step %d query %d: hash=%d bb=%d\n", step, q, na, nb);
				printf("hash:"); for(int i=0;i<na;i++) printf(" %d", ida[i]); printf("\n");
				printf("bb:  "); for(int i=0;i<nb;i++) printf(" %d", idb[i]); printf("\n");
				exit(2);
			}
		}
		for(int q = 0; q < 2; q++){
			int slot = rnd() % MAXSHAPES;
			if(!wa.ents[slot].live || !wb.ents[slot].live) continue;
			int na = collect_shape(&wa, wa.ents[slot].shape, ida);
			int nb = collect_shape(&wb, wb.ents[slot].shape, idb);
			if(na != nb || memcmp(ida, idb, na*sizeof(int))){
				printf("SHAPE MISMATCH step %d slot %d: hash=%d bb=%d\n", step, slot, na, nb);
				printf("hash:"); for(int i=0;i<na;i++) printf(" %d", ida[i]); printf("\n");
				printf("bb:  "); for(int i=0;i<nb;i++) printf(" %d", idb[i]); printf("\n");
				exit(2);
			}
		}
		if(step % 500 == 0){ printf("step %d\n", step); fflush(stdout); }
	}
	for(int wi = 0; wi < 2; wi++){
		World *w = worlds[wi];
		for(int s = 0; s < MAXSHAPES; s++) if(w->ents[s].live) remove_shape(w, s);
		for(int i = 0; i < 4; i++){ cpSpaceRemoveShape(w->space, w->segs[i]); cpShapeFree(w->segs[i]); }
		cpSpaceFree(w->space);
	}
	printf("DONE %d steps\n", N);
	return 0;
}
