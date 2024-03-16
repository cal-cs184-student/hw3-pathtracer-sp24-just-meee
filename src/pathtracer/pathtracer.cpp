#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

    PathTracer::PathTracer() {
        gridSampler = new UniformGridSampler2D();
        hemisphereSampler = new UniformHemisphereSampler3D();

        tm_gamma = 2.2f;
        tm_level = 1.0f;
        tm_key = 0.18;
        tm_wht = 5.0f;
    }

    PathTracer::~PathTracer() {
        delete gridSampler;
        delete hemisphereSampler;
    }

    void PathTracer::set_frame_size(size_t width, size_t height) {
        sampleBuffer.resize(width, height);
        sampleCountBuffer.resize(width * height);
    }

    void PathTracer::clear() {
        bvh = NULL;
        scene = NULL;
        camera = NULL;
        sampleBuffer.clear();
        sampleCountBuffer.clear();
        sampleBuffer.resize(0, 0);
        sampleCountBuffer.resize(0, 0);
    }

    void PathTracer::write_to_framebuffer(ImageBuffer& framebuffer, size_t x0,
        size_t y0, size_t x1, size_t y1) {
        sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
    }

    Vector3D
        PathTracer::estimate_direct_lighting_hemisphere(const Ray& r,
            const Intersection& isect) {
        // Estimate the lighting from this intersection coming directly from a light.
        // For this function, sample uniformly in a hemisphere.

        // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
        // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

        // make a coordinate system for a hit point
        // with N aligned with the Z direction.
        Matrix3x3 o2w;
        make_coord_space(o2w, isect.n);
        Matrix3x3 w2o = o2w.T();

        // w_out points towards the source of the ray (e.g.,
        // toward the camera if this is a primary ray)
        const Vector3D hit_p = r.o + r.d * isect.t;
        const Vector3D w_out = w2o * (-r.d);

        // This is the same number of total samples as
        // estimate_direct_lighting_importance (outside of delta lights). We keep the
        // same number of samples for clarity of comparison.
        int num_samples = scene->lights.size() * ns_area_light;
        Vector3D L_out;

        // TODO (Part 3): Write your sampling loop here
        // TODO BEFORE YOU BEGIN
        // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
        for (int i = 0; i < num_samples; i++) {

            //get the sampling direction, this is in the object space
            Vector3D w_in_object = hemisphereSampler->get_sample();
            // get the w_in in world space
            Vector3D w_in_world = o2w * w_in_object;

            //Vector3D w_in_world = w2o * w_in;
            // intersection should use the world space
            Ray ray = Ray(hit_p, w_in_world);
            ray.min_t = EPS_F;
            Intersection intersection;


            if (bvh->intersect(ray, &intersection)) {
                Vector3D l = intersection.bsdf->get_emission();
                if ((l.x == 0 && l.y == 0 && l.z == 0)) {
                    continue;
                }
                Vector3D f = isect.bsdf->f(w_out, w_in_object);

                L_out += f * l * dot(w_in_world, isect.n);
            }
        }

        L_out /= num_samples;
        L_out *= 2.0 * PI;

        return L_out;

    }

    Vector3D
        PathTracer::estimate_direct_lighting_importance(const Ray& r,
            const Intersection& isect) {
        // Estimate the lighting from this intersection coming directly from a light.
        // To implement importance sampling, sample only from lights, not uniformly in
        // a hemisphere.

        // make a coordinate system for a hit point
        // with N aligned with the Z direction.
        Matrix3x3 o2w;
        make_coord_space(o2w, isect.n);
        Matrix3x3 w2o = o2w.T();

        // w_out points towards the source of the ray (e.g.,
        // toward the camera if this is a primary ray)
        const Vector3D hit_p = r.o + r.d * isect.t;
        const Vector3D w_out = w2o * (-r.d);
        Vector3D L_out;

        // AI tool is used below to originze the code. This is an effective way to quickly make the code
        // more readable after making tons of modification.

        // Iterate over all lights in the scene
        for (SceneLight* cur_light : scene->lights) {
            // Determine the number of samples based on whether the light is a delta light
            int num_samples = ns_area_light;

            // Iterate over the number of samples
            for (int i = 0; i < num_samples; i++) {
                double distance_to_light, pdf;
                Vector3D w_in_world;

                // Sample the light source
                Vector3D sample_L = cur_light->sample_L(hit_p, &w_in_world, &distance_to_light, &pdf);
                Vector3D w_in_object = w2o * w_in_world;

                // Ensure that the sampled direction is pointing towards the surface
                if (dot(w_in_world, isect.n) >= 0) {

                    // this piece of code is from Ed
                    Ray ray = Ray(hit_p, w_in_world, distance_to_light - EPS_F);
                    ray.max_t = distance_to_light - EPS_F;
                    ray.min_t = EPS_F;

                    Intersection intersection;
                    // Check if the ray intersects any geometry in the scene
                    if (!bvh->intersect(ray, &intersection)) {
                        // Calculate the contribution of the light source using the BSDF
                        Vector3D f = isect.bsdf->f(w_out, w_in_object);
                        L_out += (sample_L * f * dot(w_in_world, isect.n)) / pdf;
                    }
                }
            }
            // Normalize the accumulated radiance by the number of samples
            L_out /= num_samples;
        }

        return L_out;

    }

    Vector3D PathTracer::zero_bounce_radiance(const Ray& r,
        const Intersection& isect) {
        // TODO: Part 3, Task 2
        // Returns the light that results from no bounces of light


        return isect.bsdf->get_emission();


    }

    Vector3D PathTracer::one_bounce_radiance(const Ray& r,
        const Intersection& isect) {
        // TODO: Part 3, Task 3
        // Returns either the direct illumination by hemisphere or importance sampling
        // depending on `direct_hemisphere_sample`
        if (direct_hemisphere_sample) {
            return estimate_direct_lighting_hemisphere(r, isect);
        }
        else {
            //std::cout << "debug222222222!" << std::endl;
            return estimate_direct_lighting_importance(r, isect);
        }


    }

    Vector3D PathTracer::at_least_one_bounce_radiance(const Ray& r,
        const Intersection& isect) {
        Matrix3x3 o2w;
        make_coord_space(o2w, isect.n);
        Matrix3x3 w2o = o2w.T();

        Vector3D hit_p = r.o + r.d * isect.t;
        Vector3D w_out = w2o * (-r.d);

        Vector3D L_out(0, 0, 0);

        // TODO: Part 4, Task 2
        // Returns the one bounce radiance + radiance from extra bounces at this point.
        // Should be called recursively to simulate extra bounces.

        L_out = one_bounce_radiance(r, isect);

        double termination_prob = 0.35;
        double continuation_prob = 1 - termination_prob;
        double pdf;
        Vector3D w_in_object;
        Vector3D sample_f = isect.bsdf->sample_f(w_out, &w_in_object, &pdf);
        if (!(coin_flip(termination_prob) || (r.depth <= 1))) {

            Vector3D w_in_world = o2w * w_in_object;
            auto ray = Ray(hit_p + (EPS_F * w_in_world), w_in_world, INF_D, r.depth - 1);
            Intersection intersection;

            if (bvh->intersect(ray, &intersection))
            {
                Vector3D next_bounce = at_least_one_bounce_radiance(ray, intersection);
                if (isAccumBounces == false) {
                    L_out = (dot(w_in_world, isect.n) * sample_f * next_bounce) / pdf / continuation_prob;
                }
                else {
                    L_out += (dot(w_in_world, isect.n) * sample_f * next_bounce) / pdf / continuation_prob;
                }
            }
            
        }
        //if (max_ray_depth == r.depth) { // for writeup
        //    L_out -= one_bounce_radiance(r, isect);
        //}
        return L_out;

    }

    Vector3D PathTracer::est_radiance_global_illumination(const Ray& r) {
        Intersection isect;
        Vector3D L_out = Vector3D(0);

        // You will extend this in assignment 3-2.
        // If no intersection occurs, we simply return black.
        // This changes if you implement hemispherical lighting for extra credit.

        // The following line of code returns a debug color depending
        // on whether ray intersection with triangles or spheres has
        // been implemented.
        //
        // REMOVE THIS LINE when you are ready to begin Part 3.

        if (!bvh->intersect(r, &isect))
            return envLight ? envLight->sample_dir(r) : L_out;

        //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);


        L_out = zero_bounce_radiance(r, isect);

        // TODO (Part 3): Return the direct illumination.
        L_out += at_least_one_bounce_radiance(r, isect);

        // TODO (Part 4): Accumulate the "direct" and "indirect"
        // parts of global illumination into L_out rather than just direct

        return L_out;
    }

    void PathTracer::raytrace_pixel(size_t x, size_t y) {
        // TODO (Part 1.2):
        // Make a loop that generates num_samples camera rays and traces them
        // through the scene. Return the average Vector3D.
        // You should call est_radiance_global_illumination in this function.

        // TODO (Part 5):
        // Modify your implementation to include adaptive sampling.
        // Use the command line parameters "samplesPerBatch" and "maxTolerance"
        int num_samples = ns_aa;          // total samples to evaluate
        Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

        Vector3D result_radiance;

        // part 5
        double s1 = 0;
        double s2 = 0;
        double total_samples = 0;

        for (int i = 0; i < num_samples; i++) {

            // part 5 check for convergence
            //std::cout << "debug222222222!" << samplesPerBatch << std::endl;
            if (i % samplesPerBatch == 0 && i != 0) {
                double mu = s1 / (total_samples / 1.0);
                double sigma = (1.0 / (total_samples - 1)) * ((s2 - ((s1 * s1) / double(total_samples))));
                double I = 1.96 * (sqrt(sigma) / sqrt(double(total_samples)));
                //std::cout << "debug222222222!" << (s2 - ((s1 * s1) / double(i))) << std::endl;
                if (I <= maxTolerance * mu) {
                    break;
                }
            }

            Vector2D sample = gridSampler->get_sample();
            double sample_x = (x + sample.x) / sampleBuffer.w;
            double sample_y = (y + sample.y) / sampleBuffer.h;

            Ray sample_ray = camera->generate_ray(sample_x, sample_y);
            sample_ray.depth = max_ray_depth;
            Vector3D cur_randance = est_radiance_global_illumination(sample_ray);
            result_radiance += cur_randance;

            total_samples++;


            // part 5 update s
            double x_k = cur_randance.illum();
            s1 += x_k;
            s2 += x_k * x_k;

        }

        result_radiance /= total_samples;

        sampleBuffer.update_pixel(result_radiance, x, y);

        sampleCountBuffer[x + y * sampleBuffer.w] = total_samples - (samplesPerBatch);

    }

    void PathTracer::autofocus(Vector2D loc) {
        Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
        Intersection isect;

        bvh->intersect(r, &isect);

        camera->focalDistance = isect.t;
    }

} // namespace CGL
