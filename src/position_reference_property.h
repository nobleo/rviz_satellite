/* Copyright 2018-2022 TomTom N.V.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#pragma once

#include "rviz/properties/tf_frame_property.h"

#include "position_reference.h"

namespace rviz
{

/**
 * RViz property that allows selecting a position reference (either NavSatFix messages or a TF frame).
 */
class RVIZ_EXPORT PositionReferenceProperty : public TfFrameProperty
{
  Q_OBJECT
public:
  explicit PositionReferenceProperty(
      const QString& name = QString(), const QString& default_value = QString(), const QString& description = QString(),
      Property* parent = nullptr, FrameManager* frame_manager = nullptr, const char* changed_slot = nullptr,
      QObject* receiver = nullptr);
  
  QString getFrame() const;
  std::string getFrameStd() const;
  
  PositionReferenceType getPositionReferenceType() const;
  
  static const QString FIX_MSG_STRING;

private Q_SLOTS:
  void adjustOptionsList();
};

}