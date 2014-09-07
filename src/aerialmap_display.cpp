/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/bind.hpp>
#include <boost/regex.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreImageCodec.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "aerialmap_display.h"

size_t replaceRegex(const boost::regex &ex, std::string& str, 
                    const std::string& replace) {
  std::string::const_iterator start = str.begin(), end = str.end();
  boost::match_results<std::string::const_iterator> what;
  boost::match_flag_type flags = boost::match_default;
  size_t count=0;
  while(boost::regex_search(start, end, what, ex, flags))
  {
    str.replace(what.position(), what.length(), replace);
    start = what[0].second;
    count++;
  }
  return count;
}

std::vector<boost::match_results<std::string::const_iterator>> 
extractRegex(const boost::regex &ex, const std::string &str) {
  std::string::const_iterator start = str.begin(), end = str.end();
  boost::match_results<std::string::const_iterator> what;
  boost::match_flag_type flags = boost::match_default;
  std::vector<boost::match_results<std::string::const_iterator> > matches;
  while(boost::regex_search(start, end, what, ex, flags))
  {
    matches.push_back(what);
    start = what[0].second;
  }
  return matches;
}

QString escapeURLString(const std::string& str) {
  return QString(QUrl(QString::fromUtf8(str.c_str())).encodedPath());
}

Ogre::TexturePtr 
textureFromBytes(const QByteArray& ba, const std::string& name) { 
  Ogre::DataStreamPtr ds;
  ds.bind(new Ogre::MemoryDataStream(ba.data(),ba.size()));
  //  check that the right codec is loaded
  char * magic = const_cast<char*>(ba.data());
  Ogre::Codec * codec = Ogre::ImageCodec::getCodec(magic,ba.size());
  if (!codec) {
    throw std::runtime_error("Failed to find image codec");
  } else {
    ROS_INFO("Loading codec: %s", codec->getType().c_str());
  }
  if (!Ogre::ImageCodec::isCodecRegistered(codec->getType())) {
    Ogre::ImageCodec::registerCodec(codec);
  }
  //  load from file in memory
  Ogre::Image img;
  img.load(ds,codec->getType());
  
  //  create texture
  const Ogre::String resGroup = 
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME; 
  
  Ogre::TextureManager& textureManager = 
      Ogre::TextureManager::getSingleton();
  Ogre::TexturePtr texture = textureManager.loadImage(name, resGroup, img);
  return texture;
}

namespace rviz
{

AerialMapDisplay::AerialMapDisplay()
  : Display()
  , manual_object_( NULL )
  , material_( 0 )
  , loaded_( false )
  , resolution_( 0.0f )
  , width_( 0 )
  , height_( 0 )
  , position_(Ogre::Vector3::ZERO)
  , orientation_(Ogre::Quaternion::IDENTITY)
  , new_coords_(false)
  , http_(0)
{
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<sensor_msgs::NavSatFix>() ),
                                          "nav_msgs::Odometry topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  alpha_property_ = new FloatProperty( "Alpha", 0.7,
                                       "Amount of transparency to apply to the map.",
                                       this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  draw_under_property_ = new Property( "Draw Behind", false,
                                       "Rendering option, controls whether or not the map is always"
                                       " drawn behind everything else.",
                                       this, SLOT( updateDrawUnder() ));

  resolution_property_ = new FloatProperty( "Resolution", 0,
                                            "Resolution of the map. (not editable)", this );
  resolution_property_->setReadOnly( true );

  width_property_ = new IntProperty( "Width", 0,
                                     "Width of the map, in meters. (not editable)", this );
  width_property_->setReadOnly( true );
  
  height_property_ = new IntProperty( "Height", 0,
                                      "Height of the map, in meters. (not editable)", this );
  height_property_->setReadOnly( true );

  position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO,
                                           "Position of the bottom left corner of the map, in meters. (not editable)",
                                           this );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY,
                                                  "Orientation of the map. (not editable)",
                                                  this );
  orientation_property_->setReadOnly( true );
}

AerialMapDisplay::~AerialMapDisplay()
{
  unsubscribe();
  clear();
}

void AerialMapDisplay::onInitialize()
{
  static int count = 0;
  std::stringstream ss;
  ss << "AerialMapObjectMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create( ss.str(),
                                                            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias( -16.0f, 0.0f );
  material_->setCullingMode( Ogre::CULL_NONE );
  material_->setDepthWriteEnabled(false);

  updateAlpha();
}

void AerialMapDisplay::onEnable()
{
  subscribe();
}

void AerialMapDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void AerialMapDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if( !topic_property_->getTopic().isEmpty() )
  {
    try
    {
      coord_sub_ = update_nh_.subscribe(
            topic_property_->getTopicStd(), 1, 
            &AerialMapDisplay::navFixCallback, 
            this);
      
      setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
    }
  }
}

void AerialMapDisplay::unsubscribe()
{
  coord_sub_.shutdown();
}

void AerialMapDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  Ogre::Pass* pass = material_->getTechnique( 0 )->getPass( 0 );
  Ogre::TextureUnitState* tex_unit = NULL;
  if( pass->getNumTextureUnitStates() > 0 )
  {
    tex_unit = pass->getTextureUnitState( 0 );
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha );

  if( alpha < 0.9998 )
  {
    material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->setDepthWriteEnabled( false );
  }
  else
  {
    material_->setSceneBlending( Ogre::SBT_REPLACE );
    material_->setDepthWriteEnabled( !draw_under_property_->getValue().toBool() );
  }
}

void AerialMapDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getValue().toBool();

  if( alpha_property_->getFloat() >= 0.9998 )
  {
    material_->setDepthWriteEnabled( !draw_under );
  }

  if( manual_object_ )
  {
    if( draw_under )
    {
      manual_object_->setRenderQueueGroup( Ogre::RENDER_QUEUE_4 );
    }
    else
    {
      manual_object_->setRenderQueueGroup( Ogre::RENDER_QUEUE_MAIN );
    }
  }
}

void AerialMapDisplay::requestFinished(int id, bool error) {
  if (error) {
    ROS_ERROR("Error loading map tile (request id %i)", id);
    return;
  }
  if (http_->currentId() == id) {
    const QByteArray data = http_->readAll();
    ROS_INFO("Finished loading request %i (%i bytes)", id, data.size());
    
    //  debug: write to disk
    QFile file("/home/gareth/test.jpg");
    file.open(QIODevice::WriteOnly);
    file.write(data);
    file.close();
    
    try {
      textureFromBytes(data,"my_sweet_texture");
    } catch (std::exception& e) {
      ROS_ERROR("Exception: %s", e.what());
    }
  }
}

void AerialMapDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  clear();
}

void AerialMapDisplay::clear()
{
  setStatus( StatusProperty::Warn, "Message", "No map received" );

  if( !loaded_ )
  {
    return;
  }

  scene_manager_->destroyManualObject( manual_object_ );
  manual_object_ = NULL;

  std::string tex_name = texture_->getName();
  texture_.setNull();
  Ogre::TextureManager::getSingleton().unload( tex_name );

  loaded_ = false;
}

void AerialMapDisplay::update( float wall_dt, float ros_dt )
{
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (new_coords_) {
      
    }
    
    //current_map_ = updated_map_;
  }

  //if (!current_map_ || !new_map_)
  if (!new_coords_)
  {
    return;
  }

//  if (current_map_->data.empty())
//  {
//    return;
//  }

  new_coords_ = false;

//  if( current_map_->info.width * current_map_->info.height == 0 )
//  {
//    std::stringstream ss;
//    ss << "AerialMap is zero-sized (" << current_map_->info.width << "x" << current_map_->info.height << ")";
//    setStatus( StatusProperty::Error, "AerialMap", QString::fromStdString( ss.str() ));
//    return;
//  }

  clear();

  setStatus( StatusProperty::Ok, "Message", "AerialMap received" );

//  ROS_DEBUG( "Received a %d X %d map @ %.3f m/pix\n",
//             current_map_->info.width,
//             current_map_->info.height,
//             current_map_->info.resolution );

  float resolution = 1;//current_map_->info.resolution;

  int width = 256;//current_map_->info.width;
  int height = 256;//current_map_->info.height;


//  Ogre::Vector3 position( current_map_->info.origin.position.x,
//                          current_map_->info.origin.position.y,
//                          current_map_->info.origin.position.z );
//  Ogre::Quaternion orientation( current_map_->info.origin.orientation.w,
//                                current_map_->info.origin.orientation.x,
//                                current_map_->info.origin.orientation.y,
//                                current_map_->info.origin.orientation.z );
  
  Ogre::Vector3 position(0,0,0);
  Ogre::Quaternion orientation(1,0,0,0);
  
  //frame_ = current_map_->header.frame_id;
  if (frame_.empty())
  {
    frame_ = "world";
  }

  // Expand it to be RGB data
  unsigned int pixels_size = width * height * 3;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 0, pixels_size);

  for (int i=0; i < height; i++) {
    for (int j=0; j < width; j++) {
      float r = (i*1.0) / height;
      float g = (j*1.0) / width;
      pixels[(i*width)*3 + j*3] = static_cast<unsigned char>(r * 255);
      pixels[(i*width)*3 + j*3 + 1] = static_cast<unsigned char>(g * 255);
    }
  }
  
  bool map_status_set = false;
  unsigned int num_pixels_to_copy = pixels_size;
  if( pixels_size != width*height*3 ) // current_map_->data.size() )
  {
    //  unreachable bullshit...
    std::stringstream ss;
   // ss << "Data size doesn't match width*height: width = " << width
   //    << ", height = " << height << ", data size = " << current_map_->data.size();
    setStatus( StatusProperty::Error, "AerialMap", QString::fromStdString( ss.str() ));
    map_status_set = true;

    // Keep going, but don't read past the end of the data.
  //  if( current_map_->data.size() < pixels_size )
  //  {
  //    num_pixels_to_copy = current_map_->data.size();
  //  }
  }

  //  What the fuck was this even for??
  // TODO: a fragment shader could do this on the video card, and
  // would allow a non-grayscale color to mark the out-of-range
  // values.
//  for( unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++ )
//  {
//    pixels[ pixel_index ] = current_map_->data[ pixel_index ];    
//  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "AerialMapTexture" << tex_count++;
  try
  {
    texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                 pixel_stream, width, height, Ogre::PF_R8G8B8, Ogre::TEX_TYPE_2D,
                                                                 0);
    
    if( !map_status_set )
    {
      setStatus( StatusProperty::Ok, "AerialMap", "AerialMap OK" );
    }
  }
  catch(Ogre::RenderingAPIException&)
  {
    Ogre::Image image;
    pixel_stream->seek(0);
    float fwidth = width;
    float fheight = height;
    if( width > height )
    {
      float aspect = fheight / fwidth;
      fwidth = 2048;
      fheight = fwidth * aspect;
    }
    else
    {
      float aspect = fwidth / fheight;
      fheight = 2048;
      fwidth = fheight * aspect;
    }

    {
      std::stringstream ss;
      ss << "AerialMap is larger than your graphics card supports.  Downsampled from [" << width << "x" << height << "] to [" << fwidth << "x" << fheight << "]";
      setStatus(StatusProperty::Ok, "AerialMap", QString::fromStdString( ss.str() ));
    }

    ROS_WARN("Failed to create full-size map texture, likely because your graphics card does not support textures of size > 2048.  Downsampling to [%d x %d]...", (int)fwidth, (int)fheight);
    //ROS_INFO("Stream size [%d], width [%f], height [%f], w * h [%f]", pixel_stream->size(), width, height, width * height);
    image.loadRawData(pixel_stream, width, height, Ogre::PF_R8G8B8);
    image.resize(fwidth, fheight, Ogre::Image::FILTER_NEAREST);
    ss << "Downsampled";
    texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
  }

  delete [] pixels;

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_->getName());
  tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "AerialMapObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject( ss2.str() );
  scene_node_->attachObject( manual_object_ );

  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( resolution*width, resolution*height, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top left
      manual_object_->position( 0.0f, resolution*height, 0.0f );
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      manual_object_->position( resolution*width, 0.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( resolution*width, resolution*height, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  manual_object_->end();

  if( draw_under_property_->getValue().toBool() )
  {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  resolution_property_->setValue( resolution );
  width_property_->setValue( width );
  height_property_->setValue( height );
  position_property_->setVector( position );
  orientation_property_->setQuaternion( orientation );

  transformAerialMap();

  loaded_ = true;

  context_->queueRender();
}

void AerialMapDisplay::navFixCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
  boost::mutex::scoped_lock lock(mutex_); /// @todo: is this necessary?
  new_coords_ = true;
  
  ref_lat_ = msg->latitude;
  ref_lon_ = msg->longitude;
  ROS_INFO("Reference point set to: %f, %f", ref_lat_, ref_lon_);
  
  //  re-load imagery
  loadImagery();
}

void AerialMapDisplay::loadImagery() {
  
  //  figure out which tile to load
  double xtile,ytile;
  const unsigned int zoom = 17;
  latLonToTileCoords(ref_lat_,ref_lon_,zoom,xtile,ytile);
  
  const unsigned int xi = std::floor(xtile);
  const unsigned int yi = std::floor(ytile);
  
  //  formulate URL
  std::string full_url = 
      "http://otile1.mqcdn.com/tiles/1.0.0/sat/{z}/{x}/{y}.jpg";
  replaceRegex(boost::regex("(http){1}s?://", boost::regex::icase),
               full_url, "");
  
  std::vector<boost::match_results<std::string::const_iterator>> url_parts = 
      extractRegex(boost::regex("[\\d\\w0-9.]+", boost::regex::icase),
                   full_url);
  if (url_parts.empty()) {
    ROS_WARN("Failed to extract base url...");
    return;
  }
  boost::match_results<std::string::const_iterator> match = url_parts.front();
  
  //  extract the host name
  std::string hostname;
  hostname.resize(match.length());
  std::copy(match[0].first, match[0].second, hostname.begin());
  full_url.erase(match.position(), match.length());
  
  ROS_INFO("Request path: %s", full_url.c_str());
  ROS_INFO("Hostname: %s", hostname.c_str());
  
  //  insert numerical values
  replaceRegex(boost::regex("\\{x\\}", boost::regex::icase),
               full_url, std::to_string(xi));
  replaceRegex(boost::regex("\\{y\\}", boost::regex::icase),
               full_url, std::to_string(yi));
  replaceRegex(boost::regex("\\{z\\}", boost::regex::icase),
               full_url, std::to_string(zoom));
  
  ROS_INFO("Request path: %s", full_url.c_str());
  
  if (!http_) {
    //  create HTTP object
    http_ = new QHttp(escapeURLString(hostname), 80, this);
    //  listen to relevant signals for loading data
    QObject::connect(http_,SIGNAL(requestFinished(int,bool)),this,
                     SLOT(requestFinished(int,bool)));
  }
  
  request_id_ = http_->get(escapeURLString(full_url), 0);
}

void AerialMapDisplay::transformAerialMap()
{
//  if (!current_map_)
//  {
//    return;
//  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  
  if (!context_->getFrameManager()->transform(frame_, ros::Time(), pose, position, orientation))
  {
    ROS_DEBUG( "Error transforming map '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), frame_.c_str(), qPrintable( fixed_frame_ ));

    setStatus( StatusProperty::Error, "Transform",
               "No transform from [" + QString::fromStdString( frame_ ) + "] to [" + fixed_frame_ + "]" );
  }
  else
  {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );
}

void AerialMapDisplay::fixedFrameChanged()
{
  transformAerialMap();
}

void AerialMapDisplay::reset()
{
  Display::reset();

  clear();
  // Force resubscription so that the map will be re-sent
  updateTopic();
}

void AerialMapDisplay::latLonToTileCoords(double lat, double lon, 
                                          unsigned int zoom, 
                                          double&x, double& y) {
  assert(zoom <= 18); /// @todo: Make this limit variable
  assert(lat > -85.0511 && lat < 85.0511);
  assert(lon > -180 && lon < 180);
  
  const double rho = M_PI / 180;
  const double lat_rad = lat * rho;
  
  unsigned int n = (1 << zoom);
  x = n * ((lon + 180) / 360.0);
  y = n * (1 - (std::log(std::tan(lat_rad) + 1/std::cos(lat_rad)) / M_PI)) / 2;
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::AerialMapDisplay, rviz::Display )
