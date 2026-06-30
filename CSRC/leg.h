#pragma once

class leg{
    private:
        float targetPos;
        float currentPos;
        float previousPos;
        float maxPositionChange;
    public: 
        void getTargetPos();
        void setTargetPos();

        void setCurrentPos();
        void getCurrentPos();

        void setPreviousPos();
        void getPreviousPos();

        void setMaxPositionChange();
        void getMaxPositionChange();
}