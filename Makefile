targets=map_manager_api.html

%.html: %.rst
	rst2html $? > $@

all: ${targets}

clean:
	rm -f *~

distclean: clean
	rm -f ${targets}
