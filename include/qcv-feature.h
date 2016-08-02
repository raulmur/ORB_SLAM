/*
 * Copyright (C) 2012 Hernan Badino <hernan.badino@gmail.com>
 *
 * This file is part of QCV
 *
 * QCV is under the terms of the GNU Lesser General Public License
 * version 2.1. See the GNU LGPL version 2.1 for details.
 * QCV is distributed "AS IS" without ANY WARRANTY, without even the
 * implied warranty of merchantability or fitness for a particular
 * purpose. 
 *
 * In no event shall the authors or contributors be liable
 * for any direct, indirect, incidental, special, exemplary, or
 * consequential damages arising in any way out of the use of this
 * software.
 *
 * By downloading, copying, installing or using the software you agree
 * to this license. Do not download, install, copy or use the
 * software, if you do not agree to this license.
 */

#ifndef __FEATURE_H
#define __FEATURE_H

/**
 *******************************************************************************
 *
 * @file Feature.h
 *
 * \class SFeature
 * \author Hernan Badino (hernan.badino@gmail.com)
 *
 * \brief Implements a data struct for modeling features.
 *
 * Implements a data struct for modeling features.
 *
 *******************************************************************************/

/* INCLUDES */
#include <vector>
#include <stdio.h>

/* CONSTANTS */

namespace QCV
{
    class SFeature
    {

    /// Public data types
    public:
        typedef enum
        {
            FS_UNINITIALIZED,
            FS_NEW,
            FS_TRACKED,
            FS_LOST
        } EFeatureState;

       static const char * FeatureStateName_v[];
       
          
    public:
        SFeature()
        {
           clear();
        }
        
        
        void print() const
        {
            printf("u: %f v: %f d: %f age: %i e: %f state: %s\n",
                   u,v,d,t,e,
		   state==FS_UNINITIALIZED?"Uninitialized":
		   state==FS_NEW?"New":
		   state==FS_TRACKED?"Tracked":
		   "Lost" );
	}

        void clear()
        {
            state = FS_UNINITIALIZED;
            idx   = -1;
            u = v = d = -1;
            t = 0;
            e = 0.;
        }

    public:
        /// u image position.
        double         u;

        /// v image position.
        double         v;

        /// d image position.
        double         d;

        /// number of times tracked.
        int            t;

        /// store user defined information (i.e. confidence).
        double         e;

        /// Frame up to which feature has been updated.
        size_t         f;

        /// General flag used for user's purposes.
        int            idx;

        /// Feature state.
        EFeatureState state;
    };

    typedef std::vector<SFeature> CFeatureVector;
}


#endif // __FEATURE_H
