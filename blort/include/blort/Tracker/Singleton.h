//SINGLETON:
//			Title:			template Singleton
//			File:				Singleton.h
//
//			Function:		Singleton template for managing resources
//
//			Author:			Thomas Mörwald
//			Date:				26.03.2009
// ----------------------------------------------------------------------------

#include <stdio.h>

namespace Tracking{

template <typename T>
class Singleton
{
public:
	static T* GetInstance(){
		if (!m_instance)
			m_instance = new T ();
		return m_instance;
	}
	virtual ~Singleton(){
		m_instance = 0;
	}
private:
	static T* m_instance;
protected:
	Singleton() { }
};

template <typename T>
T* Singleton <T>::m_instance = 0;

}

