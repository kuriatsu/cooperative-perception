#include "cooperative_perception/modelbase_planner.hpp"

despot::ValuedAction MyopicModel::Search() 
{
    despot::ValuedAction va;
    
    // if intervention requested to the target and can request more
    double decel_dist = vehicle_model_->GetDecelDistance(cp_state_->ego_speed, vehicle_model_->max_decel_, 0.0);

    /* keep request */
    if (0 < cp_state_->req_time
        && cp_state_->req_time < request_time_ 
        && cp_state_->risk_pose[cp_state_->req_target] > decel_dist) {

        va.action = cp_values_->getAction(CPValues::REQUEST, cp_state_->req_target);
        return va;
    }

    /* find request target */
    int closest_target = -1, min_dist = 100000;
    for (int i=0; i<cp_state_->risk_pose.size(); i++) {

        int is_in_history = std::count(req_target_history_.begin(), 
                                       req_target_history_.end(), 
                                       id_idx_list_[i]);

        /* progress distance while intervention request */
        double request_distance 
            = vehicle_model_->GetDecelDistance(cp_state_->ego_pose,
                                               vehicle_model_->min_decel_, 
                                               vehicle_model_->safety_margin_) 
            + vehicle_model_->yield_speed_ 
            * (request_time_ - vehicle_model_->GetDecelTime(cp_state_->ego_speed,
                                                            vehicle_model_->min_decel_));
        /* target which never requested and enough distance */
        if (is_in_history == 0 && cp_state_->risk_pose[i] > request_distance) {
            if (cp_state_->risk_pose[i] < min_dist) {

               min_dist = cp_state_->risk_pose[i];
               closest_target = i;
            }
        }
    }

    if (closest_target != -1) {
        va.action = cp_values_->getAction(CPValues::REQUEST, closest_target);
    }
    else {
        va.action = cp_values_->getAction(CPValues::NO_ACTION, 0);
    }

    return va;
}


despot::ValuedAction NoRequestModel::Search() 
{
    despot::ValuedAction va;
    va.action = cp_values_->getAction(CPValues::NO_ACTION, 0);
    return va;
}

