#pragma once

class Point {
public:
    Point() noexcept;
    void setPoint(int x, int y, int group) noexcept;
    void setGroup(int index) noexcept;
    int getX() const noexcept;
    int getY() const noexcept;
    int getGroup() const noexcept;

private:
    int x, y, group;
};