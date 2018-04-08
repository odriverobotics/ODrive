#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

extern bool user_config_loaded;
void init_configuration(void);
void save_configuration(void);
void erase_configuration(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */
