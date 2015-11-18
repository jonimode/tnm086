#include <osg/Version>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osg/PositionAttitudeTransform>
#include <osg/AnimationPath>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgUtil/Simplifier>
#include <osgUtil/Optimizer>
#include <osg/ShapeDrawable>
#include <osg/CopyOp>
#include <osgUtil/IntersectVisitor>

class IntersectRef : public osg::Referenced 
{
public:
  IntersectRef(osgUtil::IntersectionVisitor iv, osg::Light* l)
  {
    this->iv = iv;
    this->light = l;
  }
        
  osgUtil::IntersectionVisitor getVisitor()  { return this->iv; }

  osg::Light* getLight() { return this->light; }

protected:
  osgUtil::IntersectionVisitor iv;
  osg::Group* root;
  osg::Light* light;
};

class IntersectCallback : public osg::NodeCallback 
{
public:
  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    osg::ref_ptr<IntersectRef> intersectRef = 
      dynamic_cast<IntersectRef*>(node->getUserData());
    
    osgUtil::IntersectionVisitor visitor = intersectRef->getVisitor();
    node->accept(visitor);
    osg::ref_ptr<osgUtil::Intersector> lsi = visitor.getIntersector();
    
    if(lsi->containsIntersections())
    {
      intersectRef->getLight()->setDiffuse(osg::Vec4(2,2,2,1));
    }
    else
    {
      intersectRef->getLight()->setDiffuse(osg::Vec4(2,0,0,1));
    }
    lsi->reset();
    traverse(node,nv);
  }
};



int main(int argc, char *argv[]){
  
  osg::ref_ptr<osg::Group> root = new osg::Group;
  osg::StateSet* state = root->getOrCreateStateSet();
  state->setMode( GL_LIGHTING, osg::StateAttribute::ON );
  state->setMode( GL_LIGHT0, osg::StateAttribute::ON );
  state->setMode( GL_LIGHT1, osg::StateAttribute::ON );

#if 1
  /// Line ---

  osg::Vec3 line_p0 (-200, 100, 400);
  osg::Vec3 line_p1 ( 200, 300, 300);
  
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
  vertices->push_back(line_p0);
  vertices->push_back(line_p1);
  
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(0.9f,0.2f,0.3f,1.0f));

  osg::ref_ptr<osg::Geometry> linesGeom = new osg::Geometry();
  linesGeom->setVertexArray(vertices);
  linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
  
  linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,2));
  
  osg::ref_ptr<osg::Geode> lineGeode = new osg::Geode();
  lineGeode->addDrawable(linesGeom);
  lineGeode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  
  root->addChild(lineGeode);
  
  /// ---
#endif

  
  // heightfield texture
  osg::ref_ptr<osg::Texture2D> groundTexture = new osg::Texture2D(osgDB::readImageFile("ground.png"));
  // Set wrapping
  groundTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
  groundTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);

  //Create heightfield
  osg::ref_ptr<osg::HeightField> ground = new osg::HeightField(); 
  //Allocate and set interval
  ground->allocate(256,256);
  ground->setXInterval(5.0f);
  ground->setYInterval(5.0f);

  //simple ground
  for(int x = 0; x < ground->getNumRows(); x++) {
    for(int y = 0; y < ground->getNumColumns(); y++) {
        ground->setHeight(x,y, cos(x)+sin(y));
    }
  }
  osg::ref_ptr<osg::Geode> geoGround = new osg::Geode();
  geoGround->addDrawable(new osg::ShapeDrawable(ground));
  geoGround->getOrCreateStateSet()->setTextureAttributeAndModes(0, groundTexture);
  root->addChild(geoGround);
 
  //Load Plane, give it a animation Path
  osg::ref_ptr<osg::Node> cessna = osgDB::readNodeFile("cessna.osg");
  osg::ref_ptr<osg::PositionAttitudeTransform> cessnaTransform = 
      new osg::PositionAttitudeTransform();
  cessnaTransform->addChild(cessna);
  cessnaTransform->setScale(osg::Vec3(12,12,12));

  osg::ref_ptr<osg::AnimationPath> planePath = new osg::AnimationPath();
  osg::AnimationPath::ControlPoint p1(
                        osg::Vec3(128*5,128*5,764));
  p1.setScale(osg::Vec3(12,12,12));
  osg::AnimationPath::ControlPoint p2(
                        osg::Vec3(0,128*2,164));
  p2.setScale(osg::Vec3(12,12,12));
  planePath->insert(0.0f,p1);   
  planePath->insert(3.0f, p2); 
  planePath->setLoopMode( osg::AnimationPath::SWING );
  osg::ref_ptr<osg::AnimationPathCallback> planecb = new osg::AnimationPathCallback(planePath);
  cessnaTransform->setUpdateCallback(planecb);
  root->addChild(cessnaTransform);
 

  //Create dumptruck with LODs
  osg::ref_ptr<osg::Node> dumpTruck = osgDB::readNodeFile("dumptruck.osg");

  //Use 
  osgUtil::Simplifier simply(0.53);
  simply.setMaximumLength(2); 
  osg::ref_ptr<osg::Node> dumpTruckLower = 
      dynamic_cast<osg::Node*>(dumpTruck->clone(osg::CopyOp::DEEP_COPY_ALL));
  dumpTruckLower->accept(simply); 

  osg::ref_ptr<osg::Node> dumpTruckLowest = 
      dynamic_cast<osg::Node*>(dumpTruck->clone(osg::CopyOp::DEEP_COPY_ALL));
  simply.setSampleRatio(.1);
  dumpTruckLowest->accept(simply); 

  osg::ref_ptr<osg::LOD> dumpTruckLOD = new osg::LOD();
  dumpTruckLOD->setRangeMode( osg::LOD::DISTANCE_FROM_EYE_POINT );
  dumpTruckLOD->addChild(dumpTruck, 0,50);
  dumpTruckLOD->addChild(dumpTruckLower, 51,60);
  dumpTruckLOD->addChild(dumpTruckLowest,61,10000);
  
  
  osg::ref_ptr<osg::PositionAttitudeTransform> dumpTruckTransform = 
      new osg::PositionAttitudeTransform();
  
  dumpTruckTransform->addChild(dumpTruckLOD);
  dumpTruckTransform->setPosition(osg::Vec3(128*5,128*5,64));
  dumpTruckTransform->setScale(osg::Vec3(12,12,12));
  root->addChild(dumpTruckTransform);
  
  // Add Light
  osg::ref_ptr<osg::LightSource> lightS = new osg::LightSource();
  osg::ref_ptr<osg::Light> light = new osg::Light();
  light->setLightNum(0);
  light->setPosition(osg::Vec4(128*5,128*5,500.0,1.0));
  light->setDiffuse(osg::Vec4(2.0,0.0,0.0,1.0));

  lightS->setLight(light);
  root->addChild(lightS);
    
  osg::ref_ptr<osg::LightSource> light2S = new osg::LightSource();
  osg::ref_ptr<osg::Light> light2 = new osg::Light();
  light2->setLightNum(1);
  light2->setDiffuse(osg::Vec4(0.0,0.0,6.0,1.0));
  light2->setPosition(osg::Vec4(0,0,100,1.0));
  light2S->setLight(light2);
  
  osg::ref_ptr<osg::AnimationPath> lightAnim = new osg::AnimationPath();
  lightAnim->setLoopMode( osg::AnimationPath::SWING );
  lightAnim->insert(0.0f, osg::AnimationPath::ControlPoint( 
                        osg::Vec3(128*10,128*10,600)));

  lightAnim->insert(2.0f, osg::AnimationPath::ControlPoint(osg::Vec3(0,0,600)));
  osg::ref_ptr<osg::AnimationPathCallback> lightAnimC = 
                    new osg::AnimationPathCallback(lightAnim);

  osg::ref_ptr<osg::MatrixTransform> light2T = new osg::MatrixTransform();
  light2T->setUpdateCallback(lightAnimC);
  light2T->addChild(light2S);
  root->addChild(light2T);
 
  //Setup intersection callback via node
  osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = 
      new osgUtil::LineSegmentIntersector(line_p0, line_p1);
  osgUtil::IntersectionVisitor iv;
  iv.setIntersector(intersector);
  osg::ref_ptr<IntersectCallback> icb = new IntersectCallback();
  root->setUserData(new IntersectRef(iv, light));
  root->addUpdateCallback(icb);

  // Optimizes the scene-graph
  osgUtil::Optimizer optimizer;
  optimizer.optimize(root);
  
  // Set up the viewer and add the scene-graph root
  osgViewer::Viewer viewer;
 
  viewer.setSceneData(root);
  return viewer.run();
}
