#ifndef TimeoutH
#define TimeoutH TimeoutH

#include <time.h>
#include <wx/utils.h>

class CTimeout
{
public:
    explicit CTimeout( unsigned long timeoutmsec )
    {
        Start( timeoutmsec );
    }
    ~CTimeout() {}

    void Start( const unsigned long timeout_ms )
    {
        start_ = clock();
        elapse_period_ = timeout_ms * CLOCKS_PER_SEC / 1000;
        if( elapse_period_ == 0 )
        {
            elapse_period_ = 1;
        }
        end_ = start_ + elapse_period_;
    }
    unsigned char Elapsed( void ) const
    {
        return ( end_ < clock() );
    }
    void WaitTimeout( const unsigned long milliseconds ) const
    {
        wxMilliSleep( milliseconds );
    }
    unsigned long Remain( void ) const
    {
        return Elapsed() ? 0 : ( ( clock() - end_ ) * 1000 ) / CLOCKS_PER_SEC;
    }
private:
    clock_t elapse_period_;
    clock_t start_;
    clock_t end_;
};

#endif //TimeoutH
