/*
 * c_vision,
 *
 *
 * Copyright (C) 2015 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_LIB_COGNITIVE_VISION_VIEWERMANAGER_H_
#define INCLUDE_LIB_COGNITIVE_VISION_VIEWERMANAGER_H_

#include "ImageView.h"

class ViewerManager
{
    public:
        inline static ViewerManager& getInstance()
        {
            static ViewerManager instance;
            return instance;
        }

        inline void addView(const std::string& viewName)
        {
        	viewersName[viewName] = ImageView(viewName);
        }

        inline void addView(unsigned int id)
        {
        	viewersId[id] = ImageView(id);
        }

        inline ImageView& getView(const std::string& viewName)
        {
        	return viewersName[viewName];
        }

        inline ImageView& getView(unsigned int id)
        {
        	if(viewersId.count(id) == 0)
        		addView(id);
        	return viewersId[id];
        }

    private:
        ViewerManager() {};
        ViewerManager(ViewerManager const&) = delete;
        void operator=(ViewerManager const&) = delete;

    private:
        std::map<std::string, ImageView> viewersName;
        std::map<unsigned int, ImageView> viewersId;

};


#endif /* INCLUDE_LIB_COGNITIVE_VISION_VIEWERMANAGER_H_ */
