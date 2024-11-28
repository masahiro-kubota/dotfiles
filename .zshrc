
# ~/.zshrc: executed by zsh for interactive shells.

# インタラクティブシェルでない場合は終了
if [[ $- != *i* ]]; then
  return
fi

# ヒストリ設定
HISTFILE=~/.zsh_history
histsize=10000
savehist=1000000
setopt appendhistory       # 履歴を追記
setopt histignoredups      # 重複したコマンドを記録しない
setopt sharehistory        # セッション間で履歴を共有

# プロンプトの設定
PROMPT='%F{green}%n@%m%f:%F{blue}%~%f$ '

# カラー設定
if [ -x /usr/bin/dircolors ]; then
    [[ -r ~/.dircolors ]] && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
fi

# エイリアス
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

alias -g G='|grep'

# パスと環境変数
export PATH="$PATH:/home/masa/.local/bin"
export CCACHE_DIR="/var/tmp/ccache"
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
. "$HOME/.cargo/env"

# Zsh 用補完機能の読み込み
autoload -U compinit
compinit

# history begging search
bindkey "^P" history-beginning-search-backward
bindkey "^N" history-beginning-search-forward

# インクリメンタル検索のバインド
# stty -ixon
bindkey "^R" history-incremental-search-backward
bindkey "^S" history-incremental-search-forward
