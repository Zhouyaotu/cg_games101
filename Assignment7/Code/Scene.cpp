//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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
    Vector3f L_dir = 0;
    Vector3f wo = normalize(-ray.direction);
    Intersection p = Scene::intersect(ray);
    Vector3f pN = normalize(p.normal);
    if(!p.happened)
    {
        return L_dir;
    }
    if(p.m->hasEmission())
    {
        return p.m->getEmission();
    }
    Intersection lightSamplePos;
    float pdf_light;
    sampleLight(lightSamplePos,pdf_light);
    Vector3f p2light = (lightSamplePos.coords-p.coords);
    Vector3f ws = normalize(p2light);
    Intersection touchLight = Scene::intersect(Ray(p.coords, ws));
    
    if( touchLight.happened && touchLight.obj->hasEmit())
    {
        L_dir = lightSamplePos.emit * p.m->eval(wo, ws, pN)
                 * dotProduct(ws, pN) * dotProduct(ws, normalize(lightSamplePos.normal)) 
                 / dotProduct(p2light,p2light) / pdf_light;
    }
    
    Vector3f L_indir = 0;
    float rr = get_random_float();
    if(rr < RussianRoulette)
    {
        Vector3f wi = normalize(p.m->sample(wo, pN));
        Intersection touchObj = Scene::intersect(Ray(p.coords, ws));
    
        if( touchObj.happened && !touchObj.obj->hasEmit())
        {
            Vector3f wi = (touchObj.coords - p.coords).normalized();
            L_indir = castRay(Ray(p.coords, wi), 0) * p.m->eval(wo, wi, pN) * dotProduct(wi, pN) 
                        / p.m->pdf(wo, wi, pN) / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}