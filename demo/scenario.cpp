
#include <iostream>

#include "scenario.h"
#include "testtorus.h"

// My objects
#include "custom/astroidCurve.h"
#include "custom/epitrochoid.h"
#include "custom/Vivianis_curve.h"
#include "custom/B-spline_2rd_deg.h"
#include "custom/closedSubDivisionCurve.h"
#include "custom/BlendingSplineCurve.h"
#include "custom/BlendingSplineSurface.h"
#include "custom/modelSurface.h"
#include <parametrics/surfaces/gmpsphere.h>

#include "../../gmlib/modules/parametrics/surfaces/gmpplane.h"


#include "scene/sceneobjects/gmpointlightg.h"
#include "scene/sceneobjects/gmspotlightg.h"

// hidmanager
#include "hidmanager/defaulthidmanager.h"

// gmlib
#include <scene/light/gmpointlight.h>
#include <scene/sceneobjects/gmpathtrack.h>
#include <scene/sceneobjects/gmpathtrackarrows.h>

// qt
#include <QQuickItem>


template <typename T>
inline
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
  out << v.size() << std::endl;
  for(uint i=0; i<v.size(); i++) out << " " << v[i];
  out << std::endl;
  return out;
}




void Scenario::initializeScenario() {

  // Insert a light
  GMlib::Point<GLfloat,3> init_light_pos( 2.0, 4.0, 10 );
  GMlib::PointLight *light = new GMlib::PointLight(  GMlib::GMcolor::white(), GMlib::GMcolor::white(),
                                                     GMlib::GMcolor::white(), init_light_pos );
  light->setAttenuation(0.8f, 0.002f, 0.0008f);
  this->scene()->insertLight( light, false );

  // Insert Sun
  this->scene()->insertSun();

  // Default camera parameters
  int init_viewport_size = 600;
  GMlib::Point<float,3>  init_cam_pos( 0.0f, 0.0f, 0.0f );
  GMlib::Vector<float,3> init_cam_dir( 0.0f, 1.0f, 0.0f );
  GMlib::Vector<float,3> init_cam_up(  1.0f, 0.0f, 0.0f );

  // Projection cam
  auto proj_rcpair = createRCPair("Projection");
  proj_rcpair.camera->set(init_cam_pos,init_cam_dir,init_cam_up);
  proj_rcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
  proj_rcpair.camera->rotateGlobal( GMlib::Angle(-45), GMlib::Vector<float,3>( 1.0f, 0.0f, 0.0f ) );
  proj_rcpair.camera->translateGlobal( GMlib::Vector<float,3>( 0.0f, -20.0f, 20.0f ) );
  scene()->insertCamera( proj_rcpair.camera.get() );
  proj_rcpair.renderer->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );

  // How to change color
//  proj_rcpair.renderer->setClearColor(GMlib::GMcolor::rosyBrown());


  /***************************************************************************
   *                                                                         *
   * Standar example, including path track and path track arrows             *
   *                                                                         *
   ***************************************************************************/

  GMlib::Material mm(GMlib::GMmaterial::polishedBronze());
  mm.set(45.0);

//auto ptom = new TestTorus(1.0f, 0.4f, 0.6f);
//  ptom->toggleDefaultVisualizer();
//  ptom->sample(60,60,1,1);
//  this->scene()->insert(ptom);
//  auto ptrack = new GMlib::PathTrack();
//  ptrack->setLineWidth(2);
//  ptom->insert(ptrack);
//  auto ptrack2 = new GMlib::PathTrackArrows();
//  ptrack2->setArrowLength(2);
//  ptom->insert(ptrack2);

  //Add light object
  //here...


  bool showBspline = false;

  bool showSubDivisionCurve = false;

  bool showAstroid = false;
  bool showEpitrochoid = false;
  bool showVivianisCurve = false;
  bool showAnimation = false;

  bool showBlendingSurface = true;


  GMlib::DVector<GMlib::Vector<float,3>> controlPoints(5);
  controlPoints[0] = GMlib::Vector<float,3>(0, 0, 0);
  controlPoints[1] = GMlib::Vector<float,3>(3, 0, 1);
  controlPoints[2] = GMlib::Vector<float,3>(1, 0, 3);
  controlPoints[3] = GMlib::Vector<float,3>(3, 0, 5);
  controlPoints[4] = GMlib::Vector<float,3>(0, 0, 6);


  if(showBspline || showSubDivisionCurve){
      //Add control points to scene
      for(int i = 0 ; i < controlPoints.getDim() ; i++){
          auto sph = new GMlib::PSphere<float>(0.1f);
          sph->translate(controlPoints[i]);
          sph->toggleDefaultVisualizer();
          sph->sample(60,60,1,1);
          this->scene()->insert(sph);
      }
  }



  if(showBspline){
      //Using linear function B = w(t)
      auto b_spline_normal = new BSpline<float>(controlPoints, false);
      b_spline_normal->toggleDefaultVisualizer();
      b_spline_normal->sample(60,0);
      this->scene()->insert(b_spline_normal);

      //Using polynomial function of first order Blend = BPF(w(t))
      auto b_spline_blend = new BSpline<float>(controlPoints, true);
      b_spline_blend->toggleDefaultVisualizer();
      b_spline_blend->sample(60,0);
      b_spline_blend->setColor(GMlib::GMcolor::aliceBlue());
      this->scene()->insert(b_spline_blend);

      auto b_spline = new BSpline<float>(controlPoints, 4);
      b_spline->toggleDefaultVisualizer();
      b_spline->sample(60, 0);
      b_spline->setColor(GMlib::GMcolor::greenYellow());
      b_spline->showControlPoints();
      this->scene()->insert(b_spline);
  }

  if(showSubDivisionCurve){
      auto subDivCurve = new ClosedSubDivisionCurve<float>(controlPoints, 3);
      subDivCurve->toggleDefaultVisualizer();
      subDivCurve->sample(6, 0);
      subDivCurve->setColor(GMlib::GMcolor::hotPink());
      this->scene()->insert(subDivCurve);
  }


  // ModelCurve and blending curve
  if(showAstroid){
      auto astroid = new AstroidCurve<float>(2.0f, 5.0f);
      astroid->toggleDefaultVisualizer();
      astroid->sample(60,60);
      if(!showAnimation){
         this->scene()->insert(astroid);
      }

      if(showAnimation){
          auto bls_astroid = new BlendingSpline<float>(astroid, 7);
          bls_astroid->toggleDefaultVisualizer();
          bls_astroid->sample(200, 0);
          //bls_astroid->showControlCurves();
          this->scene()->insert(bls_astroid);
      }

  }

  if(showEpitrochoid){
      auto epi = new Epitrochid<float>(2.0f, 5.0f);
      epi->toggleDefaultVisualizer();
      epi->sample(200,0);
      if(!showAnimation){
         this->scene()->insert(epi);
      }

      if(showAnimation){
          auto bls_epi = new BlendingSpline<float>(epi, 11, 1);
          bls_epi->toggleDefaultVisualizer();
          bls_epi->sample(200, 0);
          //bls_epi->showControlCurves();
          this->scene()->insert(bls_epi);
      }
  }

  if(showVivianisCurve){
      auto vivianis = new VivianisCurve<float>(5.0f);
      vivianis->toggleDefaultVisualizer();
      vivianis->sample(60,60);
      if(!showAnimation){
         this->scene()->insert(vivianis);
      }

      if(showAnimation){
          auto bls_vivianis = new BlendingSpline<float>(vivianis, 11);
          bls_vivianis->toggleDefaultVisualizer();
          bls_vivianis->sample(200, 0);
          bls_vivianis->showControlCurves();
          this->scene()->insert(bls_vivianis);
      }
  }

  if(showBlendingSurface){
      int const sampleU = 20;
      int const sampleV = 20;

//      auto copy = new GMlib::PTorus<float>(1.0f, 0.4f, 0.6f);
//      auto copy = new GMlib::PPlane<float>(GMlib::Point<float, 3>(0.0f, 0.0f, 0.0f), GMlib::Vector<float, 3>(10.0f, 0.0f, 0.0f), GMlib::Vector<float, 3>(0.0f, 0.0f, 10.0f));
      auto copy = new ModelSurface<float>(3.0f);


      auto bls_surf = new BlendingSurface<float>(copy, 4, 4, sampleU, sampleV);
      bls_surf->toggleDefaultVisualizer();
      bls_surf->sample(sampleU, sampleV);
      bls_surf->showControlSurf();
      this->scene()->insert(bls_surf);



  }





}




void Scenario::cleanupScenario() {

}




void Scenario::callDefferedGL() {

  GMlib::Array< const GMlib::SceneObject*> e_obj;
  this->scene()->getEditedObjects(e_obj);

  for(int i=0; i < e_obj.getSize(); i++)
    if(e_obj(i)->isVisible()) e_obj[i]->replot();
}

