#ifndef OutputState_h__
#define OutputState_h__

#include <string>

typedef enum OutputState{
	TRUE, FALSE, UNKNOWN
}OutputState;

class trilean{
protected:
	OutputState state;
public:

	trilean(OutputState _state = UNKNOWN) :state(_state){
	}

	static bool isUnknown(OutputState a){
		return (a == UNKNOWN) ? true : false;
	}

	static std::string tostring(const trilean& obj){
		if (obj.state == TRUE)
			return "TRUE";
		else if (obj.state == FALSE)
			return "FALSE";
		else
			return "UNKNOWN";
	}

	explicit operator OutputState() const {
		return this->state;
	}

	trilean operator&(const trilean& rhs){
		return trilean((
			this->state == FALSE | rhs.state == FALSE) ? FALSE : (this->state == UNKNOWN | rhs.state == UNKNOWN) ? UNKNOWN : TRUE);
	}

	trilean operator&&(const trilean& rhs){
		return *this & rhs;
	}

	trilean& operator=(const trilean& rhs){
		this->state = rhs.state;
		return *this;
	}

	bool operator==(const trilean& rhs){
		return (this->state == rhs.state) ? true : false;
	}

	trilean operator|(const trilean& rhs){
		return trilean((
			this->state == TRUE | rhs.state == TRUE) ? TRUE : (this->state == UNKNOWN | rhs.state == UNKNOWN) ? UNKNOWN : FALSE);
	}

	trilean operator^(const trilean& rhs){
		return trilean((
			this->state == FALSE | rhs.state == FALSE) ? TRUE : (this->state == UNKNOWN | rhs.state == UNKNOWN) ? UNKNOWN : FALSE);
	}

	trilean operator!(){
		return trilean(
			(this->state == UNKNOWN) ? UNKNOWN : (this->state == FALSE) ? TRUE : FALSE);
	}

	bool operator!=(const trilean& rhs){
		return (this->state == rhs.state)? false : true;
	}


};

std::ostream& operator<<(std::ostream& os, const trilean& obj){
	os << trilean::tostring(obj);
	return os;
}

/* 3 state logic functions */
trilean AND_3(trilean a, trilean b){
	return a & b;
}
trilean NAND_3(trilean a, trilean b){
	return !(a&b);
}
trilean OR_3(trilean a, trilean b){
	return a | b;
}
trilean XOR_3(trilean a, trilean b){
	return a^b;
}
trilean NOT_3(trilean a){
	return !a;
}

#endif // OutputState_h__