/**
 * @file drive.h
 * @author Augusto De Conto (augustodeconto@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DRIVE_H_INCLUDED
#define DRIVE_H_INCLUDED

class Drive
{
    public:
    enum class Channel
    {
        Motor1,
        Motor2,
    };

    static void preInitSafety(void);

    int   setup(void);
    int   setDutyCycle(Channel channel, float duty);
    float getDutyCycle(Channel channel);

    private:
    float dutyCommand[2];
};

#endif /* DRIVE_H_INCLUDED */
