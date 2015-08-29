#pragma once

namespace cold {
    namespace contract {
        struct disposable {
            virtual void dispose() = 0;
        protected:
            ~disposable() = default;
        };

        struct handle : public disposable {
            virtual void set_location(
                double yaw_, double pitch_, double roll_,
                double x_, double y_, double z_) = 0;

            virtual void deactivate() = 0;

            virtual void activate() = 0;

            virtual bool is_active() const = 0;
        protected:
            ~handle() = default;
        };

        struct world : public disposable {
            virtual handle* add_mesh(
                int triangle_count_, double* triangles_, int tri_stride_,
                int indices_count, int* indices_, int ind_stride_) = 0;
        protected:
            ~world() = default;
        };
    }
}
