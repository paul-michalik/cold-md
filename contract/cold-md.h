#pragma once

namespace cold {
    namespace contract {
        struct disposable {
            virtual void dispose() = 0;
        protected:
            ~disposable() = default;
        };

        struct world : public disposable {
            virtual world* add_object() = 0;
            
        protected:
            ~world() = default;
        };
    }
}
