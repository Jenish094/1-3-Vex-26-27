#include "main.h"
#include "pros/apix.h"
#include "liblvgl/lvgl.h"
#include "autoSelect/selection.h"
#include <cstring>

namespace selector{

int auton;
int autonCount;
const char *btnmMap[] = {"","","","","","","","","","",""}; // up to 10 autons

lv_obj_t *tabview;
lv_obj_t *redBtnm;
lv_obj_t *blueBtnm;

void redBtnmAction(lv_event_t * e){
	//printf("red button: %s released\n", txt);
	lv_obj_t *btnm = (lv_obj_t *)lv_event_get_target(e);
	const char *txt = lv_buttonmatrix_get_button_text(btnm, lv_buttonmatrix_get_selected_button(btnm));

	for(int i = 0; i < autonCount; i++){
		if(strcmp(txt, btnmMap[i]) == 0){
			auton = i+1;
		}
	}
}

void blueBtnmAction(lv_event_t * e)
{
	//printf("blue button: %s released\n", txt);
	lv_obj_t *btnm = (lv_obj_t *)lv_event_get_target(e);
	const char *txt = lv_buttonmatrix_get_button_text(btnm, lv_buttonmatrix_get_selected_button(btnm));

	for(int i = 0; i < autonCount; i++){
		if(strcmp(txt, btnmMap[i]) == 0){
			auton = -(i+1);
		}
	}
}

void skillsBtnAction(lv_event_t * e){
  //printf("skills pressed");
	auton = 0;
}

int tabWatcher() {
	uint32_t activeTab = lv_tabview_get_tab_active(tabview);
	while(1){
		uint32_t currentTab = lv_tabview_get_tab_active(tabview);

		if(currentTab != activeTab){
			activeTab = currentTab;
			if(activeTab == 0){
				if(auton == 0) auton = 1;
				auton = abs(auton);
				lv_buttonmatrix_set_selected_button(redBtnm, abs(auton)-1);
			}else if(activeTab == 1){
				if(auton == 0) auton = -1;
				auton = -abs(auton);
				lv_buttonmatrix_set_selected_button(blueBtnm, abs(auton)-1);
			}else{
				auton = 0;
			}
		}

		pros::delay(20);
	}
}

void init(int hue, int default_auton, const char **autons){

	int i = 0;
	do{
		memcpy(&btnmMap[i], &autons[i], sizeof(&autons));
		i++;
	}while(strcmp(autons[i], "") != 0);

	autonCount = i;
	auton = default_auton;

	// lvgl theme
	// Note: Theme functions have changed in LVGL v9
	// lv_theme_t *th = lv_theme_default_init(NULL, lv_color_hex(hue), lv_color_hex(hue), false, &lv_font_default);
	// lv_disp_set_theme(NULL, th);

	// create a tab view object
	tabview = lv_tabview_create(lv_screen_active());

	// add 3 tabs (the tabs are page (lv_page) and can be scrolled
	lv_obj_t *redTab = lv_tabview_add_tab(tabview, "Red");
	lv_obj_t *blueTab = lv_tabview_add_tab(tabview, "Blue");
	lv_obj_t *skillsTab = lv_tabview_add_tab(tabview, "Skills");

	//set default tab
	if(auton < 0){
		lv_tabview_set_active(tabview, 1, LV_ANIM_OFF);
	}else if(auton == 0){
		lv_tabview_set_active(tabview, 2, LV_ANIM_OFF);
	}

	// add content to the tabs
	// button matrix
	redBtnm = lv_buttonmatrix_create(redTab);
	lv_buttonmatrix_set_map(redBtnm, btnmMap);
	lv_obj_add_event_cb(redBtnm, redBtnmAction, LV_EVENT_CLICKED, NULL);
	lv_buttonmatrix_set_selected_button(redBtnm, abs(auton)-1);//3
	lv_obj_set_size(redBtnm, 450, 50);
	lv_obj_set_pos(redBtnm, 0, 100);
	lv_obj_align(redBtnm, LV_ALIGN_CENTER, 0, 0);

	// blue tab
	blueBtnm = lv_buttonmatrix_create(blueTab);
	lv_buttonmatrix_set_map(blueBtnm, btnmMap);
	lv_obj_add_event_cb(blueBtnm, blueBtnmAction, LV_EVENT_CLICKED, NULL);
	lv_buttonmatrix_set_selected_button(blueBtnm, abs(auton)-1);
	lv_obj_set_size(blueBtnm, 450, 50);
	lv_obj_set_pos(blueBtnm, 0, 100);
	lv_obj_align(blueBtnm, LV_ALIGN_CENTER, 0, 0);

	// skills tab
	lv_obj_t *skillsBtn = lv_button_create(skillsTab);
	lv_obj_t *label = lv_label_create(skillsBtn);
	lv_label_set_text(label, "Skills");
	lv_obj_add_event_cb(skillsBtn, skillsBtnAction, LV_EVENT_CLICKED, NULL);
	// lv_btn_set_state(skillsBtn, LV_BTN_STATE_TGL_REL);
	lv_obj_set_size(skillsBtn, 450, 50);
	lv_obj_set_pos(skillsBtn, 0, 100);
	lv_obj_align(skillsBtn, LV_ALIGN_CENTER, 0, 0);

	// start tab watcher
	pros::Task tabWatcher_task(tabWatcher);

}

} // namespace selector