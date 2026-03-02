//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <algorithm>


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    
    Intersection inter_p,inter_light;
    inter_p = intersect(ray);
    if (!inter_p.happened) return Vector3f(0, 0, 0);

    if (inter_p.m == nullptr) return Vector3f(0, 0, 0);

    if(inter_p.m->hasEmission()) return inter_p.m->getEmission();

    float pdf_light;
    sampleLight(inter_light,pdf_light);
    
    Vector3f p = inter_p.coords, x = inter_light.coords;
    Vector3f ws = (x-p).normalized();

    Vector3f wo = ray.direction;
    Vector3f N = inter_p.normal.normalized();
    Ray p_to_x(p, ws);

    Vector3f L_dir(0,0,0);
    Intersection block_check = intersect(p_to_x);
    if (pdf_light > 0.000001f) {
        if(block_check.happened && block_check.distance - (x-p).norm() > -0.001f){
            float cos_theta = std::max(0.0f, dotProduct(ws, N));
            float cos_theta_x = std::max(0.0f, dotProduct(-ws, inter_light.normal.normalized()));
            L_dir = inter_light.emit 
            * inter_p.m->eval(wo, ws, N) 
            * cos_theta
            * cos_theta_x
            / dotProduct(x-p, x-p) 
            / pdf_light;
        }
    }

    Vector3f L_indir(0,0,0);
    if(get_random_float() <= RussianRoulette){
        Vector3f wi = inter_p.m->sample(wo,N).normalized();
        Ray p_to_obj(p,wi);
        Intersection hit_obj = intersect(p_to_obj);
        if(hit_obj.happened && hit_obj.m != nullptr && !hit_obj.m->hasEmission()){
            if(inter_p.m->pdf(wo,wi,N)>0.0001f){
                float cos_theta_obj = std::max(0.0f, dotProduct(wi, N));
                L_indir = castRay(p_to_obj,depth+1) 
                * inter_p.m->eval(wo,wi,N) 
                * cos_theta_obj
                / inter_p.m->pdf(wo,wi,N) 
                / RussianRoulette;
            }
        }
    }
    return L_dir + L_indir; 

}