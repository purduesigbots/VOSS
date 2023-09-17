#include "legs/controller/AbstractController.hpp"

namespace legs::controller {

AbstractController::AbstractController(localizer::AbstractLocalizer& l) {
	this->l = &l;
}

} // namespace legs::controller