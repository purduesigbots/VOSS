#include "voss/selector/Selector.hpp"
#include "liblvgl/lvgl.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <unordered_map>

namespace voss::selector {

std::unordered_map<int, const char*> autonNameLookup;

pros::Mutex auton_mtx;

lv_obj_t* redLbl;
lv_obj_t* blueLbl;
lv_obj_t* skillsLbl;

lv_obj_t* redBtnm;
lv_obj_t* blueBtnm;

int auton;
const char* btnmMap[11] = {"", "", "", "", "", "", "", "", "", "", ""};

void update_text() {
	const char* color;
	const char* autonName;
	if (auton != 0) {
		color = (auton > 0) ? "Red" : "Blue";
		autonName = autonNameLookup[abs(auton)];
	} else {
		color = "";
		autonName = "Skills";
	}

	lv_label_set_text_fmt(redLbl, "Currently running: %s %s", color, autonName);
	lv_label_set_text_fmt(blueLbl, "Currently running: %s %s", color, autonName);
	lv_label_set_text_fmt(skillsLbl, "Currently running: %s %s", color,
	                      autonName);
}

void init(int default_auton, const char** autons) {

	int ptr = 0;
	int i = 0;
	do {
		memcpy(&btnmMap[ptr], &autons[ptr], sizeof(&autons));
		if (strcmp(autons[ptr], "\n") == 0) {
			ptr++;
			continue;
		}
		autonNameLookup[i + 1] = autons[ptr];
		ptr++;
		i++;
	} while (strcmp(autons[ptr], "") != 0);

	autonNameLookup[0] = "Skills";

	// Lock mutex while constructing selector
	auton_mtx.take();

	auton = default_auton;

	lv_theme_t* th =
	    lv_theme_default_init(lv_disp_get_default(), lv_color_hex(0xCFB53B),
	                          lv_color_hex(0xCFB53B), true, LV_FONT_DEFAULT);

	/*Create a Tab view object*/
	lv_obj_t* tabview;
	tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 50);

	/*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
	lv_obj_t* redTab = lv_tabview_add_tab(tabview, "Red");
	lv_obj_t* blueTab = lv_tabview_add_tab(tabview, "Blue");
	lv_obj_t* skillsTab = lv_tabview_add_tab(tabview, "Skills");

	// set default tab
	if (auton < 0) {
		lv_tabview_set_act(tabview, 1, LV_ANIM_OFF);
	} else if (auton == 0) {
		lv_tabview_set_act(tabview, 2, LV_ANIM_OFF);
	}

	lv_style_t lbl_style;
	lv_style_init(&lbl_style);
	lv_style_set_border_width(&lbl_style, 1);
	lv_style_set_border_color(&lbl_style, th->color_primary);
	// lv_style_set_pad_all(&lbl_style, 5);

	/*Add content to the tabs*/
	redLbl = lv_label_create(redTab);
	lv_obj_add_style(redLbl, &lbl_style, 0);
	lv_obj_align(redLbl, LV_ALIGN_CENTER, 0, -60);

	blueLbl = lv_label_create(blueTab);
	lv_obj_add_style(blueLbl, &lbl_style, 0);
	lv_obj_align(blueLbl, LV_ALIGN_CENTER, 0, -60);

	skillsLbl = lv_label_create(skillsTab);
	lv_obj_add_style(skillsLbl, &lbl_style, 0);
	lv_obj_align(skillsLbl, LV_ALIGN_CENTER, 0, -60);

	update_text();

	// Release mutex
	auton_mtx.give();

	redBtnm = lv_btnmatrix_create(redTab);
	lv_btnmatrix_set_map(redBtnm, btnmMap);
	lv_btnmatrix_set_one_checked(redBtnm, true);
	lv_obj_set_size(redBtnm, 450, 50);
	lv_obj_set_pos(redBtnm, 0, 100);
	lv_obj_align(redBtnm, LV_ALIGN_CENTER, 0, 0);
	lv_btnmatrix_set_btn_ctrl_all(redBtnm, LV_BTNMATRIX_CTRL_CHECKABLE);
	if (auton > 0) {
		lv_btnmatrix_set_btn_ctrl(redBtnm, abs(auton) - 1,
		                          LV_BTNMATRIX_CTRL_CHECKED);
	}
	lv_obj_add_event_cb(
	    redBtnm,
	    [](lv_event_t* e) {
		    lv_event_code_t code = lv_event_get_code(e);
		    lv_obj_t* obj = lv_event_get_target(e);
		    if (code == LV_EVENT_VALUE_CHANGED) {
			    uint32_t id = lv_btnmatrix_get_selected_btn(obj);
			    auton_mtx.take();
			    auton = id + 1;
			    update_text();
			    auton_mtx.give();
			    lv_btnmatrix_clear_btn_ctrl_all(blueBtnm, LV_BTNMATRIX_CTRL_CHECKED);
		    }
	    },
	    LV_EVENT_ALL, NULL);

	/*Add content to the tabs*/
	blueBtnm = lv_btnmatrix_create(blueTab);
	lv_btnmatrix_set_map(blueBtnm, btnmMap);
	lv_btnmatrix_set_one_checked(blueBtnm, true);
	lv_obj_set_size(blueBtnm, 450, 50);
	lv_obj_set_pos(blueBtnm, 0, 100);
	lv_obj_align(blueBtnm, LV_ALIGN_CENTER, 0, 0);
	lv_btnmatrix_set_btn_ctrl_all(blueBtnm, LV_BTNMATRIX_CTRL_CHECKABLE);
	if (auton < 0) {
		lv_btnmatrix_set_btn_ctrl(blueBtnm, abs(auton) - 1,
		                          LV_BTNMATRIX_CTRL_CHECKED);
	}
	lv_obj_add_event_cb(
	    blueBtnm,
	    [](lv_event_t* e) {
		    lv_event_code_t code = lv_event_get_code(e);
		    lv_obj_t* obj = lv_event_get_target(e);
		    if (code == LV_EVENT_VALUE_CHANGED) {
			    uint32_t id = lv_btnmatrix_get_selected_btn(obj);
			    auton_mtx.take();
			    auton = -1 * (id + 1);
			    update_text();
			    auton_mtx.give();
			    lv_btnmatrix_clear_btn_ctrl_all(redBtnm, LV_BTNMATRIX_CTRL_CHECKED);
		    }
	    },
	    LV_EVENT_ALL, NULL);

	lv_obj_t* skillsBtn = lv_btn_create(skillsTab);
	lv_obj_t* label = lv_label_create(skillsBtn);
	lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
	lv_label_set_text(label, "Skills");
	lv_obj_set_size(skillsBtn, 450, 50);
	lv_obj_set_pos(skillsBtn, 0, 100);
	lv_obj_align(skillsBtn, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_event_cb(
	    skillsBtn,
	    [](lv_event_t* e) {
		    lv_event_code_t code = lv_event_get_code(e);
		    lv_obj_t* obj = lv_event_get_target(e);
		    if (code == LV_EVENT_PRESSED) {
			    auton_mtx.take();
			    auton = 0;
			    update_text();
			    auton_mtx.give();
			    lv_btnmatrix_clear_btn_ctrl_all(redBtnm, LV_BTNMATRIX_CTRL_CHECKED);
			    lv_btnmatrix_clear_btn_ctrl_all(blueBtnm, LV_BTNMATRIX_CTRL_CHECKED);
		    }
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