#include <iostream>

#include <gtkmm.h>
#include <gdkmm.h>

class InfoItem:public Gtk::Box{
	private:
	Gtk::Label* nameLabel;
	Gtk::Label* valueLabel;

	public:
	InfoItem(std::string name);

	void setName(std::string name);
	std::string getName();

	void setValue(bool value);
    void setValue(int value);
    void setValue(long value);
    void setValue(float value);
    void setValue(double value);
    void setValue(std::string value);

	bool getValueAsBool();
    int getValueAsInt();
    long getValueAsLong();
    float getValueAsFloat();
    double getValueAsDouble();
    std::string getValue();
};
