#ifndef FRAMETOGRAPH_H
#define FRAMETOGRAPH_H

#include <vector>

#include "scenegraph.h"
#include "asstcommon.h"
using namespace std;
class FrameToGraph:public SgNodeVisitor{
private:	
	vector<RigTForm> nodes_;
	int counter;
public:
  FrameToGraph(vector<RigTForm>& nodes) : nodes_(nodes), counter(0) {}

  virtual bool visit(SgTransformNode& node) {
	 std::shared_ptr<SgRbtNode> rbtPtr = std::dynamic_pointer_cast<SgRbtNode>(node.shared_from_this());
    if (rbtPtr){
		RigTForm n = nodes_.at(counter);
		rbtPtr->setRbt(n);
		counter++;
		//nodes_.erase(nodes_.begin());
	}
    return true;
  }
 };

#endif