#include "voss/selector/Selector.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <unordered_map>

namespace voss::selector {

std::unordered_map<const char*, int> autonMap;

/*Create a Tab view object*/
lv_obj_t* tabview;

pros::Mutex auton_mtx;

int auton;
int autonCount;
const char* btnmMap[11] = {"", "", "", "", "", "", "", "", "", "", ""};

void init(int hue, int default_auton, const char** autons) {

	int i = 0;
	do {
		memcpy(&btnmMap[i], &autons[i], sizeof(&autons));
		autonMap[autons[i]] = i + 1;
		i++;
	} while (strcmp(autons[i], "") != 0);

	autonCount = i;
	auton_mtx.take();
	auton = default_auton;
	auton_mtx.give();

	tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 50);

	/*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
	lv_obj_t* redTab = lv_tabview_add_tab(tabview, "Red");
	lv_obj_t* blueTab = lv_tabview_add_tab(tabview, "Blue");
	lv_obj_t* skillsTab = lv_tabview_add_tab(tabview, "Skills");

	// set default tab
	auton_mtx.take();
	if (auton < 0) {
		lv_tabview_set_act(tabview, 1, LV_ANIM_OFF);
	} else if (auton == 0) {
		lv_tabview_set_act(tabview, 2, LV_ANIM_OFF);
	}
	auton_mtx.give();

	/*Add content to the tabs*/
	lv_obj_t* redBtnm = lv_btnmatrix_create(redTab);
	lv_btnmatrix_set_map(redBtnm, btnmMap);
	lv_obj_set_size(redBtnm, 450, 50);
	lv_obj_set_pos(redBtnm, 0, 100);
	lv_obj_align(redBtnm, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_event_cb(
	    redBtnm,
	    [](lv_event_t* e) {
		    lv_event_code_t code = lv_event_get_code(e);
		    lv_obj_t* obj = lv_event_get_target(e);
		    if (code == LV_EVENT_VALUE_CHANGED) {
			    uint32_t id = lv_btnmatrix_get_selected_btn(obj);
			    const char* txt = lv_btnmatrix_get_btn_text(obj, id);
			    auton_mtx.take();
			    auton = autonMap[txt];
			    auton_mtx.give();
		    }
	    },
	    LV_EVENT_ALL, NULL);

	/*Add content to the tabs*/
	lv_obj_t* blueBtnm = lv_btnmatrix_create(blueTab);
	lv_btnmatrix_set_map(blueBtnm, btnmMap);
	lv_obj_set_size(blueBtnm, 450, 50);
	lv_obj_set_pos(blueBtnm, 0, 100);
	lv_obj_align(blueBtnm, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_event_cb(
	    blueBtnm,
	    [](lv_event_t* e) {
		    lv_event_code_t code = lv_event_get_code(e);
		    lv_obj_t* obj = lv_event_get_target(e);
		    if (code == LV_EVENT_VALUE_CHANGED) {
			    uint32_t id = lv_btnmatrix_get_selected_btn(obj);
			    const char* txt = lv_btnmatrix_get_btn_text(obj, id);
			    auton_mtx.take();
			    auton = -1 * autonMap[txt];
			    auton_mtx.give();
		    }
	    },
	    LV_EVENT_ALL, NULL);

	lv_obj_t* skillsBtn = lv_btn_create(skillsTab);
	lv_obj_t* label = lv_label_create(skillsBtn);
	lv_label_set_text(label, "Skills");
	lv_obj_set_size(skillsBtn, 450, 50);
	lv_obj_set_pos(skillsBtn, 0, 100);
	lv_obj_align(skillsBtn, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_event_cb(
	    skillsBtn,
	    [](lv_event_t* e) {
		    auton_mtx.take();
		    auton = 0;
		    auton_mtx.give();
	    },
	    LV_EVENT_ALL, NULL);
}

int get_auton() {
	auton_mtx.take();
	int ret = auton;
	auton_mtx.give();
	return ret;
}

} // namespace voss::selector