/*
 * PropertyMap_FIX.h
 *
 *  Created on: Nov 29, 2012
 *      Author: acmarkus
 */

#ifndef PROPERTYMAP_FIX_H_
#define PROPERTYMAP_FIX_H_


#include <mv_camera/PropertyMap.h>

namespace mv_camera
{
template<typename ContainerAllocator>
  const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> PropertyMapRequest_<
      ContainerAllocator>::SHOW_FLAGS = "flagsOn";
template<typename ContainerAllocator>
  const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> PropertyMapRequest_<
      ContainerAllocator>::SHOW_VALUES = "valuesOn";
}



#endif /* PROPERTYMAP_FIX_H_ */
