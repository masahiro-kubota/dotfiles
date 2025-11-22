
# ~/.zshrc: executed by zsh for interactive shells.

# インタラクティブシェルでない場合は終了
if [[ $- != *i* ]]; then
  return
fi

HISTFILE=~/.zsh_history
HISTSIZE=10000
SAVEHIST=10000
setopt appendhistory          # 履歴を追記
setopt inc_append_history      # コマンド実行後に履歴を即保存
setopt sharehistory           # セッション間で履歴を共有
setopt histignoredups         # 重複する履歴を無視

# プロンプトの設定
PROMPT='%F{green}%n@%m%f:%F{blue}%~%f$ '


# カラー設定
if [ -x /usr/bin/dircolors ]; then
    [[ -r ~/.dircolors ]] && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
fi

# エイリアス
if [ -f ~/.aliases ]; then
    . ~/.aliases
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

# history begging search 前方一致限定の履歴検索
bindkey "^P" history-beginning-search-backward
bindkey "^N" history-beginning-search-forward

# インクリメンタル検索のバインド
stty -ixon
bindkey "^R" history-incremental-search-backward
bindkey "^S" history-incremental-search-forward

# smart insert last word 直前のコマンドの最後の単語を一瞬で入力できる
autoload smart-insert-last-word
zle -N insert-lastword smart-insert-last-word
bindkey '^[.' insert-lastword

# 入力中のコマンドをviで編集できる。
autoload -Uz edit-command-line
zle -N edit-command-line
bindkey '^[e' edit-command-line

# 入力中のコマンドを一瞬どかすことができる
bindkey '^U' push-line

# Git worktree selector
select_worktree() {
  local worktrees
  worktrees=$(git worktree list --porcelain | awk '/worktree / {print $2}')
  if [[ -z "$worktrees" ]]; then
    echo "No worktrees found."
    return 1
  fi
  local selected
  selected=$(echo "$worktrees" | fzf)
  if [[ -n "$selected" ]]; then
    echo "Switching to: $selected"
    cd "$selected"
  fi
}

# Ctrl+J でworktree選択
zle -N select_worktree
bindkey '^J' select_worktree

[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh

# Directory Hash ディレクトリ名のエイリアスのようなもの
hash -d dot=/home/masa/Documents/dotfiles

setopt auto_cd

export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion




#eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"
#. "$HOME/.local/bin/env"

# ros2 setting
source /opt/ros/humble/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
export GTEST_COLOR=1
eval "$(register-python-argcomplete3 colcon)"
eval "$(register-python-argcomplete ros2)"
alias srcws='source install/setup.zsh && eval "$(register-python-argcomplete3 ros2)" && eval "$(register-python-argcomplete3 colcon)"'

export DOTNET_ROOT="/usr/local/share/dotnet"
export PATH=$PATH:$DOTNET_ROOT

export PATH=$PATH:~/bin


function start_new_task() {
    local task_id=$1
    # mainを更新
    cd /workspace/main
    git pull origin main
    # 新しいworktreeを作成
    git worktree add ../task-${task_id} -b ai/task-${task_id}
    cd ../task-${task_id}
    git submodule update --init --recursive
    echo "Ready to work on task ${task_id}"
}


if [ -f /usr/share/fzf/key-bindings.zsh ]; then
  source /usr/share/fzf/key-bindings.zsh
elif [ -f /usr/share/doc/fzf/examples/key-bindings.zsh ]; then
  source /usr/share/doc/fzf/examples/key-bindings.zsh
fi
bindkey -e
bindkey '^R' fzf-history-widget

# セキュリティ診断＋HTMLレポート生成コマンド
nmapsec() {
  if [ -z "$1" ]; then
    echo "Usage: nmapsec <IP-or-hostname>"
    return 1
  fi

  local target="$1"
  local base_dir="$HOME/nmapsec_logs"

  mkdir -p "$base_dir/$target"
  local ts
  ts="$(date +'%Y%m%d-%H%M%S')"
  local prefix="$base_dir/$target/scan-sec-${target}-${ts}"

  echo "[*] Scanning $target ..."
  sudo nmap -sS -sV -sC --script vuln -O -T4 -oA "$prefix" "$target"

  echo "[*] Generating HTML report ..."
  xsltproc "${prefix}.xml" -o "${prefix}.html"

  echo "[*] Opening ${prefix}.html"
  xdg-open "${prefix}.html" >/dev/null 2>&1 &

  echo "[*] Done. Files are in: $base_dir/$target"
}

ensure_ssh_config_entry_by_remote() {
  local user_part="$1"
  local host_part="$2"
  local keyfile="$HOME/.ssh/id_ed25519"
  local ssh_config="$HOME/.ssh/config"

  # config が無ければ作る
  if [ ! -f "$ssh_config" ]; then
    touch "$ssh_config"
    chmod 600 "$ssh_config"
  fi

  # (HostName, User) が一致するブロックがあるかチェック
  local entry_exists
  entry_exists=$(awk -v h="$host_part" -v u="$user_part" '
    tolower($1) == "host" {
      in_block = 1
      host_ok = 0
      user_ok = 0
      next
    }
    in_block && tolower($1) == "hostname" && $2 == h { host_ok = 1 }
    in_block && tolower($1) == "user"     && $2 == u { user_ok = 1 }

    # 空行でブロック終了
    in_block && NF == 0 {
      if (host_ok && user_ok) {
        print "yes"
        exit
      }
      in_block = 0
    }
    END {
      if (in_block && host_ok && user_ok) {
        print "yes"
      }
    }
  ' "$ssh_config")

  if [ "$entry_exists" = "yes" ]; then
    return 0  # 既にあるので何もしない
  fi

  # --- 無いので alias を聞いて追加 ---
  echo "→ ~/.ssh/config にこの組み合わせがありません："
  echo "   HostName = $host_part"
  echo "   User     = $user_part"
  printf "登録する Host 名（エイリアス）を入力（例: rk04）: "
  read -r alias_name

  if [ -n "$alias_name" ]; then
    {
      echo ""
      echo "Host $alias_name"
      echo "  HostName $host_part"
      echo "  User $user_part"
      echo "  IdentityFile $keyfile"
    } >> "$ssh_config"
    chmod 600 "$ssh_config"
    echo "✔ Host '$alias_name' を登録しました"
    echo "  → 次回から: ssh $alias_name / myssh $alias_name で接続できます"
  fi
}


###############################################
#             myssh 関数本体
###############################################
myssh() {
  if [ "$#" -ne 1 ]; then
    echo "Usage: myssh user@host | myssh alias"
    return 1
  fi

  local arg="$1"
  local keyfile="$HOME/.ssh/id_ed25519"
  local pubfile="${keyfile}.pub"
  local target
  local local_user="$USER"
  local local_host
  local_host="$(hostname -I | awk '{print $1}')"

  # --- 鍵がなければ作る ---
  if [ ! -f "$pubfile" ]; then
    echo "→ SSH鍵がありません。ed25519鍵を作成します..."
    ssh-keygen -t ed25519 -f "$keyfile" -C "$USER@$(hostname)" -N ""
    echo "✔ SSH鍵を作成しました：$pubfile"
  fi

  # ==========================
  #   引数の解釈
  # ==========================
  if [[ "$arg" == *@* ]]; then
    # --- user@host モード（初回登録用） ---
    local user_part host_part
    user_part="${arg%%@*}"
    host_part="${arg##*@}"

    # config に (HostName, User) があるか確認・なければ追加
    ensure_ssh_config_entry_by_remote "$user_part" "$host_part"

    target="${user_part}@${host_part}"
  else
    # --- alias or host モード ---
    local ssh_config="$HOME/.ssh/config"

    if [ -f "$ssh_config" ] && grep -qE "^[[:space:]]*Host[[:space:]]+$arg( |\$)" "$ssh_config"; then
      # ~/.ssh/config に Host <arg> がある → alias とみなす
      target="$arg"
      # alias の場合は登録処理はしない
    else
      # Host にも無い → host とみなして $USER@host で登録モード
      local user_part host_part
      user_part="$USER"
      host_part="$arg"
      ensure_ssh_config_entry_by_remote "$user_part" "$host_part"
      target="${user_part}@${host_part}"
    fi
  fi

  # --- 公開鍵を読み込み ---
  local pubkey
  pubkey="$(< "$pubfile")"

  # --- alias ファイル送信 ---
  #    target が "user@host" でも "alias" でも OK
  scp -q ~/.kubota_aliases ~/.kubota_remote_aliases "${target}:~/." || return 1

  # --- リモートで rcfile を作って bash 起動 ---
  ssh "$target" -t "
    export MY_PUBKEY='$pubkey'
    export PULL_LOCAL_USER='$local_user'
    export PULL_LOCAL_HOST='$local_host'

    cat > ~/.kubota_rc << 'EOF'
if [ -f ~/.bashrc ]; then . ~/.bashrc; fi
if [ -f ~/.kubota_aliases ]; then . ~/.kubota_aliases; fi
if [ -f ~/.kubota_remote_aliases ]; then . ~/.kubota_remote_aliases; fi
EOF
    exec bash --rcfile ~/.kubota_rc -i
  "
}

