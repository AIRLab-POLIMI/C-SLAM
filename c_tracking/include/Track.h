/*
 * c_tracking,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_tracking.
 *
 * c_tracking is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_tracking is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_tracking.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TRACK_H_
#define TRACK_H_

#include "CMT.h"

#include <string>

class Track : public CMT
{
public:
	inline void setId(const uint64_t& id)
	{
		this->id = id;
	}

	inline uint64_t getId() const
	{
		return id;
	}

	inline void setLabel(const std::string& label)
	{
		this->label = label;
	}

	inline const std::string& getLabel() const
	{
		return label;
	}

private:
	std::string label;
	uint64_t id;
};



#endif /* TRACK_H_ */
