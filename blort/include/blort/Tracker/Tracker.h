/**
 * @file Tracker.h
 * @author Thomas Mörwald & Bence Magyar
 * @date April 2012
 * @version 0.2
 * @brief Main file of Tracker.
 * @namespace Tracking
 */
#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <opencv2/core/core.hpp>
#include <blort/Tracker/headers.h>
#include <blort/Tracker/Timer.h>
#include <blort/Tracker/Resources.h>
#include <blort/Tracker/Distribution.h>
#include <blort/Tracker/Predictor.h>
#include <blort/TomGine/tgMathlib.h>
#include <blort/Tracker/CDataFile.h>
#include <blort/Tracker/ModelEntry.h>
#include <blort/TomGine/tgLighting.h>

/** @brief namespace Tracking */
namespace Tracking{

  const float pi = 3.14159265358979323846f;

  typedef std::vector<ModelEntry*> ModelEntryList;

  /** @brief Main class of Tracker, defining API */
  class Tracker{
  public:
    struct Parameter{
      TomGine::tgCamera::Parameter camPar;
      int model_id_count;
      int hypotheses_id_count;
      int num_particles;					// number of particles to draw for each frame
      int num_recursions;					// number of recursions for each image
      unsigned hypotheses_trials;			// number of trials a hypothesis gets for convergence, until comparisson to original
      int convergence;					// convergence factor
      float edge_tolerance;				// maximal angular deviation of edges to match in degrees
      unsigned int num_spreadings;		// number of spreadings during image processing
      unsigned int m_spreadlvl;			// Width of edges in pixels (automatically adjusted)
      Particle variation;					// standard deviation of particle distribution in meter
      float minTexGrabAngle;				// Angular threshold between view vector and face normal for grabing texture
      unsigned int max_kernel_size;		// Max. edgel comparison kernel (distance of neighbouring pixels taken into account)
      int kernel_size;					// Edgel comparison kernel (distance of neighbouring pixels taken into account)
      std::string modelPath;				// Path to model recources
      std::string texturePath;			// Path to texture recources
      std::string shaderPath;				// Path to shader recources
      float model_sobel_th;				// Threshold for sobel edge detection for model
      float image_sobel_th;				// Threshold for sobel edge detection for image
      float c_th_base;					// Base confidence threshold for tracking quality detection
      float c_th_min;						// Minimum confidence threshold for tracking quality detection
      float c_th_fair;					// Fair confidence threshold for tracking quality detection
      float c_mv_not;						// Threshold for no movement detection
      float c_mv_slow;					// Threshold for slow movement detection
      float c_th_lost;					// Threshold for tracking lost detection
      float pred_no_convergence;			// Part of particles of distribution voting for no convergence

      // Constructor (assigning default values)
      Parameter();
    };

  public:
    Tracker();
    ~Tracker();

    // Main functions (init, image_processing, tracking, reset)
    /** @brief Initialize tracker with an INI file and image/window width and height in pixel */
    bool init(const char* trackINIFile, const char* camCalFile, const char* poseCalFile);
    bool init(const Parameter& trackParam);

    /** @brief Perform image processing with edge detection */
    void loadImage(unsigned char* image, GLenum format=GL_BGR);
    /** @brief Perform image processing with edge detection
  *	@param image image data
  *	@param format data format */
    virtual void image_processing(unsigned char* image, GLenum format=GL_BGR)=0;
    /** @brief Perform image processing with edge detection, painting a virtual object into the image
  *	@param image image data
  *	@param model geometry of virtual object
  *	@param pose position and orientation of the virtual object in world space
  *	@param format data format */
    virtual void image_processing(unsigned char* image, const TomGine::tgModel &model, const TomGine::tgPose &pose, GLenum format=GL_BGR)=0;
    /** @brief Perform image processing with edge detection, painting the model with id as virtual object into image
  *	@param image image data
  *	@param model_id id of the model
  *	@param pose position and orientation of the virtual object in world space
  *	@param format data format*/
    virtual void image_processing(unsigned char* image, int model_id, const TomGine::tgPose &pose, GLenum format=GL_BGR)=0;

    /** @brief Tracks all models by matching their edges against edges of images */
    virtual bool track()=0;
    /** @brief Tracks model by id by matching their edges against edges of images
  *	@param id the id of the model given by addModel() or addModelFromFile() */
    virtual bool track(int id)=0;

    /** @brief Resets the pose of all models to the initial pose */
    void reset();
    /** @brief Resets the pose of a model to the initial pose
  *	@param id the id of the model given by addModel() or addModelFromFile() */
    void reset(int id);

    /** @brief Uses the reset function but also locks, calls track() and unlocks the tracker so every confidence will get updated */
    void resetUnlockLock();

    // Model handling
    /** @brief Adds a geometrical model to the tracker
  *	@return id of the added model (-1 if not successfull)	*/
    int addModel(TomGine::tgModel& m, TomGine::tgPose& pose,  std::string label, bool bfc=true);

    /** @brief Adds a geometrical model from file (ply-fileformat) to the tracker
  *	@param filename absolute filename of the model (or relative to the execution path)
  *	@param pose place where the model is initially put to
  *	@param label label of the model
  *	@param bfc enable/disable backfaceculling (look up OpenGL Backface Culling)
  *	@return  id of the added model (-1 if not successfull)	*/
    int addModelFromFile(const char* filename, TomGine::tgPose& pose, std::string label, bool bfc=true);

    /** @brief Remove model from tracker by id */
    void removeModel(int id);

    //BENCE: never called
    //	/** @brief Adds a pose hypothesis to model */
    //	void addPoseHypothesis(int id, TomGine::tgPose &p, std::string label, bool bfc);

    /** @brief DO NOT USE THIS FUNCTION! */
    ModelEntry* getModelEntry(int id);

    /** @brief Get current pose of a model */
    void getModelPose(int id, TomGine::tgPose& p);

    /** @brief Get low pass filtered pose of a model */
    void getModelPoseLPF(int id, TomGine::tgPose& p);

    /** @brief Get current pose of a model in camera coordinates */
    void getModelPoseCamCoords(int id, TomGine::tgPose& p);

    /** @brief Get current velocity of a model ( in m/s and rad/s ) */
    void getModelVelocity(int id, float &translational, float &angular);

    /** @brief Get movement state of a model */
    void getModelMovementState(int id, movement_state &m);

    /** @brief Get a tracking quality state of a model */
    void getModelQualityState(int id, quality_state &q);

    /** @brief Get a tracking quality state of a model */
    void getModelConfidenceState(int id, confidence_state &q);

    /** @brief Get the initial pose of a model */
    void getModelInitialPose(int id, TomGine::tgPose& p);

    /** @brief Get mean confidence value of distribution of a model */
    void getModelConfidence(int id, float& c);

    /** @brief Get confidence value of a model at current pose (edge based comparison)*/
    void getModelConfidenceEdge(int id, float& c);

    /** @brief Get confidence value of a model at current pose (color based comparison)*/
    void getModelConfidenceColor(int id, float& c);

    /** @brief Get 3D point from 2D window coordinates */
    bool getModelPoint3D(int id, int x_win, int y_win, double& x3, double& y3, double& z3);

    /** @brief Calculates mask produced by the geometry edges of a model */
    void getModelMask(int id, Texture &mask);

    /** @brief Calculates mask produced by the geometry edges of a model */
    void getModelEdgeMask(int id, Texture &mask);

    /** @brief Gets flag to mask geometry edges of model */
    bool getModelMaskOwnEdges(int id);

    /** @brief Set the initial pose of a model */
    void setModelInitialPose(int id, TomGine::tgPose& p);

    /** @brief Set a model predictor */
    void setModelPredictor(int id, Predictor* predictor);

    /** @brief Locks the model with id */
    void setModelLock(int id, bool lock);

    /** @brief Sets the number of recursions and particles used for tracking */
    void setModelRecursionsParticle(int id, int num_recursions, int num_particles);

    /** @brief Set the number of particles (in percent) which are voting for no convergence (for capturing fast movement)*/
    void setModelPredictorNoConvergence(int id, float no_conv);

    /** @brief Sets mask for a model to define edges which are not considered for the model */
    void setModelMask(int id, Texture *mask=0);

    /** @brief Sets flag to mask geometry edges of model */
    void setModelMaskOwnEdges(int id, bool masked);

    /** @brief Save model to file */
    void saveModel(int id, const char* pathname);
    /** @brief Save all models to file */
    void saveModels(const char* pathname);

    /** @brief Takes screenshot and saves it to file */
    void saveScreenshot(const char* filename);

    /** @brief Returns the rendered image */
    cv::Mat getImage();

    /** @brief Captures image and attaches it to model as texture */
    virtual void textureFromImage(bool force=false){}
    virtual void textureFromImage(int id, const TomGine::tgPose &pose, bool use_num_pixels=true){}

    /** @brief Remove textures from model */
    virtual void untextureModels(){}

    // Drawing to screen (result, ...)
    /** @brief Draw all models */
    virtual void drawResult(float linewidth=1.0f)=0;
    void drawModel(TomGine::tgPose p);
    void drawModel(const TomGine::tgModel &m, const TomGine::tgPose &p);
    virtual void drawTrackerModel(int id, const TomGine::tgPose &p, float linewidth=1.0f){};
    void drawModelWireframe(const TomGine::tgModel &m, const TomGine::tgPose &p, float linewidth=1.0f);
    void drawCoordinateSystem(float linelength=0.5f, float linewidth=1.0f, TomGine::tgPose pose=TomGine::tgPose());
    void drawCoordinates(float linelength=1.0f);
    void drawImage(unsigned char* image);
    void drawPixel(float u, float v, vec3 color=vec3(1.0,1.0,1.0), float size=1.0f);
    void drawPoint(float x, float y, float z, float size=1.0);
    void drawCalibrationPattern(float point_size=1.0f);
    void loadCalibrationPattern(const char* mdl_file);
    void drawTest();
    void printStatistics();

    // set parameters for texture tracking
    virtual void setKernelSize(int val){ }
    virtual void setEdgeShader(){ }
    virtual void setColorShader(){ }

    // Set Parameters
    void setFrameTime(double dTime);
    bool setCameraParameters(TomGine::tgCamera::Parameter cam_par);
    void setSpreadLvl(unsigned int val){ params.m_spreadlvl = val; }

    // Get Parameters
    float getCamZNear(){ return m_cam_perspective.GetZNear(); }
    float getCamZFar(){ return m_cam_perspective.GetZFar(); }
    unsigned int getSpreadLvl(){ return params.m_spreadlvl; }

    // get Flags
    bool getLockFlag(){ return m_lock; }
    bool getEdgesImageFlag(){ return m_draw_edges; }
    bool getDrawParticlesFlag(){ return m_showparticles; }
    int  getModelModeFlag(){ return m_showmodel; }

    // set Flags
    void setLockFlag(bool val);
    void setEdgesImageFlag(bool val){ m_draw_edges = val; }
    void setDrawParticlesFlag(bool val){ m_showparticles = val; }
    void setModelModeFlag(int val){ m_showmodel = val; }

    // Functions for analysing PDF
    virtual void evaluatePDF( int id,
                              float x_min, float y_min,
                              float x_max, float y_max,
                              int res,
                              const char* meshfile, const char* xfile){}

    virtual std::vector<float> getPDFxy(	Particle pose,
                                          float x_min, float y_min,
                                          float x_max, float y_max,
                                          int res,
                                          const char* filename=NULL, const char* filename2=NULL){ std::vector<float> a; return a;}

    virtual void savePDF(	std::vector<float> vPDFMap,
                          float x_min, float y_min,
                          float x_max, float y_max,
                          int res,
                          const char* meshfile, const char* xfile){}

    const Parameter getParams(){ return params; }

  protected:
    Parameter params;

    // Resources
    Texture* m_tex_frame;
    std::vector<Texture*> m_tex_frame_ip;

    std::vector<vec3> m_calib_points;

    TomGine::tgLighting m_lighting;
    ImageProcessor* m_ip;

    float m_ftime;
    Timer m_timer;

    // ModelEntry
    ModelEntryList m_modellist;
    ModelEntryList m_hypotheses;

    TomGine::tgCamera m_cam_perspective;
    TomGine::tgCamera m_cam_default;

    // Flags
    bool m_lock;
    bool m_showparticles;
    int  m_showmodel;
    bool m_draw_edges;
    bool m_tracker_initialized;
    bool m_drawimage;

    // Functions
    /** @brief Load parameter of tracker with an INI file */
    bool loadTrackParsFromINI(const char* inifile);
    bool loadCamParsFromINI(const char* camCalFile, const char* poseCalFile);

    void getGlError();

    void computeModelEdgeMask(ModelEntry* modelEntry, Texture &mask);

    virtual bool initInternal()=0;
    bool init();
    bool initGL();

  };

} // namespace Tracking

#endif
