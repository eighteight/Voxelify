
#include "c4d.h"
#include "c4d_symbols.h"
#include "c4d_tools.h"
#include "lib_splinehelp.h"
#include "ge_dynamicarray.h"
#include "voxelify.h"
#include <vector>
#include <string>
#include <iostream>
#include "Voxelifyer.h"
#include "VGrid.h"
// unique ID obtained from www.plugincafe.com
#define ID_VOXELIFY 1031321

typedef std::pair<SplineObject*,Real> SplinePair;
bool comparator ( const SplinePair& l, const SplinePair& r){
    return l.second > r.second;
}

class Voxelify : public ObjectData
{
private:
    Real maxSeg, minSeg;
    Matrix parentMatrix;
    SplineObject* ComputeSpline(BaseThread* bt, vector<VGrid> grids, LONG longestPercent, GeDynamicArray<GeDynamicArray<Vector> > &splinesAtPoint);
    void DoRecursion(BaseObject *op, BaseObject *child, GeDynamicArray<Vector> &points, Matrix ml);
    vector<vector<float> > objectPointsToPoints(GeDynamicArray<Vector>  objectPoints);
    Voxelifier vox;
    
public:
    virtual SplineObject* GetContour(BaseObject *op, BaseDocument *doc, Real lod, BaseThread *bt);
    virtual Bool Init(GeListNode *node);
    static NodeData *Alloc(void) { return gNew Voxelify; }
};

Bool Voxelify::Init(GeListNode *node)
{
	BaseObject		*op   = (BaseObject*)node;
	BaseContainer *data = op->GetDataInstance();
    
	//data->SetReal(CTTSPOBJECT_MAXSEG,1000.);
	//data->SetReal(CTTSPOBJECT_MINSEG,0.1);
    data->SetLong(SPLINEOBJECT_INTERPOLATION,SPLINEOBJECT_INTERPOLATION_ADAPTIVE);
    GePrint("Voxelify by http://twitter.com/eight_io for Cinema 4D r14");
    
    return TRUE;
}

void Voxelify::DoRecursion(BaseObject *op, BaseObject *child, GeDynamicArray<Vector> &points, Matrix ml) {
	BaseObject *tp;
	if (child){
		tp = child->GetDeformCache();
		ml = ml * child->GetMl();
		if (tp){
			DoRecursion(op,tp,points,ml);
		}
		else{
			tp = child->GetCache(NULL);
			if (tp){
				DoRecursion(op,tp,points,ml);
			}
			else{
				if (!child->GetBit(BIT_CONTROLOBJECT)){
					if (child->IsInstanceOf(Opoint)){
						PointObject * pChild = ToPoint(child);
						LONG pcnt = pChild->GetPointCount();
						const Vector *childVerts = pChild->GetPointR();
						for(LONG i=0; i < pcnt; i++){
							points.Push(childVerts[i] * ml * parentMatrix);
						}
					}
				}
			}
		}
		for (tp = child->GetDown(); tp; tp=tp->GetNext()){
			DoRecursion(op,tp,points,ml);
		}
	}
}

vector<vector<float> > Voxelify::objectPointsToPoints(GeDynamicArray<Vector>  objectPoints){
    vector<vector<float> > points(objectPoints.GetCount());
    for (LONG i = 0; i < objectPoints.GetCount(); i++) {
        vector<float> p(3);
        p[0] = objectPoints[i].x;
        p[1] = objectPoints[i].y;
        p[2] = objectPoints[i].z;
        points[i] = p;
    }
    return points;
}



SplineObject* Voxelify::GetContour(BaseObject *op, BaseDocument *doc, Real lod, BaseThread *bt){
    BaseContainer *data = op->GetDataInstance();
    BaseObject* parent=(BaseObject*)data->GetLink(CTT_OBJECT_LINK,doc,Obase);
    if (!parent) return NULL;
    
    LONG startObject = data->GetLong(START_FRAME);
    LONG endObject = data->GetLong(END_FRAME);
    
    if (startObject >=endObject) return NULL;
    
    maxSeg = data->GetReal(CTTSPOBJECT_MAXSEG,30.);
    minSeg = data->GetReal(CTTSPOBJECT_MINSEG);
    
    LONG delta = data->GetLong(OBJECT_SKIP,1);
    delta = delta < 1 ? 1 : delta;
    
    GeDynamicArray<BaseObject*> children;
    GeDynamicArray<GeDynamicArray<Vector> > splineAtPoint;
    
    BaseObject* chld = NULL;
    LONG trck = 0;
    for (chld=parent->GetDownLast(); chld; chld=chld->GetPred()) {
        if (trck >= startObject && trck<= endObject && trck % delta == 0){
            children.Push((BaseObject*)chld->GetClone(COPYFLAGS_NO_HIERARCHY|COPYFLAGS_NO_ANIMATION|COPYFLAGS_NO_BITS,NULL));
        }
        trck++;
    }
    
    if (children.GetCount() < 2) {
        //return NULL;
    }
    
    LONG splineInterpolation = data->GetLong(SPLINEOBJECT_INTERPOLATION);
    LONG longestPercent = data->GetLong(TAKE_LONGEST, 1);
    longestPercent = longestPercent > 100 ? 100: longestPercent;
    
    
    GeDynamicArray<GeDynamicArray<Vector> > objectPoints(children.GetCount());
	StatusSetBar(0);
    StatusSetText("Collecting Points");
    vector<vector<float> > points;
    std::vector<VGrid> grids;

    int gridSize = 20;
    for (int k= 0; k < children.GetCount(); k++){
        Vector bb = children[k]->GetRad();        
        Matrix ml;
        DoRecursion(op,children[k],objectPoints[k], ml);
        points = objectPointsToPoints(objectPoints[k]);
        GePrint(children[k]->GetName());
        grids.push_back(vox.voxelify(points,bb.x/(float)gridSize,bb.y/(float)gridSize,bb.z/(float)gridSize));
    }

    parentMatrix = parent->GetMl();
    
    SplineObject* parentSpline = ComputeSpline(bt, grids, longestPercent, splineAtPoint);
    
    ModelingCommandData mcd;
    mcd.doc = doc;
    mcd.op = parentSpline;
    
    if(!SendModelingCommand(MCOMMAND_JOIN, mcd)){
        return NULL;
    }
    
    SplineObject* ret = ToSpline(mcd.result->GetIndex(0L));
    
    ret->GetDataInstance()->SetLong(SPLINEOBJECT_INTERPOLATION, splineInterpolation);
    
    for (int k=0; k<children.GetCount(); k++){
        if (children[k]){
            BaseObject::Free(children[k]);
        }
    }
    
    return ret;
Error:
    for (int i = 0; i < children.GetCount(); i++){
        BaseObject::Free(children[i]);
    }
    return NULL;
}

SplineObject* Voxelify::ComputeSpline(BaseThread* bt, vector<VGrid> grids, LONG longestPercent, GeDynamicArray<GeDynamicArray<Vector> > &splinesAtPoint){
    
    StatusSetBar(5);
    StatusSetText("Connecting Points");
    
    SplineObject* parentSpline = SplineObject::Alloc(0, SPLINETYPE_BSPLINE);
    
    std::vector<SplinePair >splinePairs;
    
    Real avSplineSize = 0.0, avSplineLength = 0.0;
    GeDynamicArray<GeDynamicArray<LONG> > validPoints(grids.size());
    for (LONG k=0; k < grids.size(); k++){
        validPoints[k] = GeDynamicArray<LONG>(grids[0].points.size());
        validPoints[k].Fill(0,grids[0].points.size(),1);

    }
    
    GeDynamicArray<LONG> indxs;

    for (LONG i = 0; i < grids[0].points.size(); i++) {
        if (grids[0].indices[i] == -1) continue;
        GeDynamicArray<Vector> rawSpline;
        Vector point(grids[0].points[i][0], grids[0].points[i][1], grids[0].points[i][2]);
        indxs.Push(i);
        rawSpline.Push(point);
        splinesAtPoint.Push(rawSpline);
    }

    Real distMin = std::numeric_limits<float>::max();
    Real distMax = 0.;
    AutoAlloc<SplineHelp> splineHelp;
    LONG i, o;
    for (i = 0; i < splinesAtPoint.GetCount(); i++){//iterate points
        bool lastPointCaptured = true;
        LONG indx = indxs[i];
        for (o=0; o < grids.size()-1; o++){ // for each point iterate objects and collect nearest points
            LONG closestIndx = grids[o+1].indices[indx];
            if ( closestIndx == -1){
                GePrint("error finding neighbor "+LongToString(o)+"/"+LongToString(i));
                if (o == grids.size()-1){
                    lastPointCaptured = false;
                }
                continue;
            }
            Real dist = hypot(grids[o].points[indx][0] - grids[o+1].points[indx][0], hypot(grids[o].points[indx][1] - grids[o+1].points[indx][1], grids[o].points[indx][2] - grids[o+1].points[indx][2]));
            distMin = distMin < dist ? distMin : dist;
            distMax = distMax > dist ? distMax : dist;
            
            if (o != grids.size()-1) {
                //if (dist > maxSeg || dist < minSeg) {
                //    continue;
                //}
            }
            validPoints[o][i] = 0;
            
            Vector clsst(grids[o+1].points[i][0],grids[o+1].points[i][1],grids[o+1].points[i][2]);
            
            if (splinesAtPoint[i].Find(clsst) == NOTOK){
                splinesAtPoint[i].Push(clsst);
            }
        }
        
        if (!lastPointCaptured) continue;
        SplineObject* spline=SplineObject::Alloc(splinesAtPoint[i].GetCount(),SPLINETYPE_BSPLINE);
        if (!spline) continue;
        
        spline->GetDataInstance()->SetBool(SPLINEOBJECT_CLOSED, FALSE);
        
        Vector *padr = spline->GetPointW();
        for (LONG l = 0; l < splinesAtPoint[i].GetCount(); l++){
            padr[l] = splinesAtPoint[i][l];
        }
        
        splineHelp->InitSpline(spline);
        Real splnLength = splineHelp->GetSplineLength();
        if (splnLength > 0.0){
            splinePairs.push_back(SplinePair(spline, splnLength));
            avSplineLength += splnLength;
            avSplineSize += splinesAtPoint[i].GetCount();
        } else {
            SplineObject::Free(spline);
        }
        
        if (i % 5 == 0){
            LONG progress = 10 + (90*i)/splinesAtPoint.GetCount();
            StatusSetBar(progress);
            StatusSetText(LongToString(progress)+"%");
            if (bt && bt->TestBreak()){
                //break; //this break seems to be kicking in randomly killing the loop
            }
        }
    }
    
    LONG splnSize = splinePairs.size();
    GePrint(LongToString(i)+" points "+LongToString(splnSize)+" splines");
    if (splnSize > 0) {
        LONG limit =  splnSize * longestPercent / 100;
        limit = limit == 0 ? 1 : limit;
        
        std::sort(splinePairs.begin(), splinePairs.end(),comparator);
        
        for (int s = 0; s < limit; s++){
            avSplineLength += splinePairs[s].second;
            splinePairs[s].first->InsertUnder(parentSpline);
        }
        
        String sizeAvg = splinesAtPoint.GetCount() == 0? "Nan":RealToString(avSplineSize/splinesAtPoint.GetCount());
        
        GePrint("d="+RealToString(distMin)+" : "+RealToString(distMax)+" avSpln="+RealToString(avSplineLength/avSplineSize));
    }
	StatusSetText(LongToString(i)+" points "+LongToString(splnSize)+" splines");
    
    if (splnSize == 0) return NULL;
    return parentSpline;
}


Bool Registervoxelify(void)
{
	return RegisterObjectPlugin(ID_VOXELIFY ,GeLoadString(IDS_VOXELIFY),OBJECT_GENERATOR|OBJECT_INPUT|OBJECT_ISSPLINE|OBJECT_CALL_ADDEXECUTION,Voxelify::Alloc,"voxelify",AutoBitmap("tsp.tif"),0);
}
