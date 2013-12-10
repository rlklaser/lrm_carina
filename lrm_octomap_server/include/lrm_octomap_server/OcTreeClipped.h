/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file ClippingOcTree.h
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 9, 2013
 *
 */

#ifndef CLIPPINGOCTREE_H_
#define CLIPPINGOCTREE_H_

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <ctime>

namespace octomap {

// node definition
class OcTreeNodeClipped: public OcTreeNode {

public:
	OcTreeNodeClipped() :
			OcTreeNode(), is_clipped(false) {
	}

	OcTreeNodeClipped(const OcTreeNodeClipped& rhs) :
			OcTreeNode(rhs), is_clipped(false) {
	}

	bool operator==(const OcTreeNodeClipped& rhs) const {
		return (rhs.value == value && rhs.is_clipped == is_clipped);
	}

	// children
	inline OcTreeNodeClipped* getChild(unsigned int i) {
		return static_cast<OcTreeNodeClipped*>(OcTreeNode::getChild(i));
	}
	inline const OcTreeNodeClipped* getChild(unsigned int i) const {
		return static_cast<const OcTreeNodeClipped*>(OcTreeNode::getChild(i));
	}

	bool createChild(unsigned int i) {
		if (children == NULL)
			allocChildren();
		children[i] = new OcTreeNodeClipped();
		return true;
	}

	void setClipped(bool value) {
		is_clipped = value;
	}

	bool isClipped() {
		return is_clipped;
	}

protected:
	bool is_clipped;
};

// tree definition
class OcTreeClipped: public OccupancyOcTreeBase<OcTreeNodeClipped> {

public:
	/// Default constructor, sets resolution of leafs
	OcTreeClipped(double resolution) :
			OccupancyOcTreeBase<OcTreeNodeClipped>(resolution) {
	}

	/// virtual constructor: creates a new object of same type
	/// (Covariant return type requires an up-to-date compiler)
	OcTreeClipped* create() const {
		return new OcTreeClipped(resolution);
	}

	std::string getTreeType() const {
		return "OcTreeClipped";
	}

	virtual void updateNodeClipping(const OcTreeKey& key, float log_odds_update, bool clip) {

		if (log_odds_update == 0)
			return;

		OcTreeNodeClipped* node = this->search(key);
		if (node) {

			if(node->isClipped())
				return;

			this->updateNode(key, log_odds_update);

			if (log_odds_update < 0) {
				if (node->getLogOdds() >= this->getClampingThresMaxLog()) {
					//return;
					node->setClipped(clip);
					ROS_INFO_STREAM("node clipped " << node->getMeanChildLogOdds() );
				}
			} else {
				if (node->getLogOdds() <= this->getClampingThresMinLog()) {
					//return;
					//node->setClipped(clip);
				}
			}
		}
		else {
			this->updateNode(key, log_odds_update);
		}
	}

protected:
	/**
	 * Static member object which ensures that this OcTree's prototype
	 * ends up in the classIDMapping only once
	 */
	class StaticMemberInitializer {
	public:
		StaticMemberInitializer() {
			OcTreeStamped* tree = new OcTreeStamped(0.1);
			AbstractOcTree::registerTreeType(tree);
		}
	};
/// to ensure static initialization (only once)
	static StaticMemberInitializer ocTreeClippedMemberInit;

};

} // end namespace

#endif /* CLIPPINGOCTREE_H_ */
